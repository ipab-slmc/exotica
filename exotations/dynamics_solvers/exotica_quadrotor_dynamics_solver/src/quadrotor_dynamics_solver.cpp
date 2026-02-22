//
// Copyright (c) 2019, Wolfgang Merkt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_quadrotor_dynamics_solver/quadrotor_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("QuadrotorDynamicsSolver", exotica::QuadrotorDynamicsSolver)

namespace exotica
{
QuadrotorDynamicsSolver::QuadrotorDynamicsSolver()
{
    num_positions_ = 6;
    num_velocities_ = 6;
    num_controls_ = 4;

    J_.setZero();
    J_.diagonal() = Eigen::Vector3d(0.0023, 0.0023, 0.004);

    J_inv_.setZero();
    J_inv_.diagonal() = Eigen::Vector3d(1. / 0.0023, 1. / 0.0023, 1. / 0.0040);
}

void QuadrotorDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const int num_positions_in = scene_in->GetKinematicTree().GetNumControlledJoints();
    // TODO: This is a terrible check (not against name etc.), but can stop _some_ mismatches between URDF/model and dynamics
    if ((num_positions_in) != 6 || scene_in->GetKinematicTree().GetControlledBaseType() != BaseType::FLOATING)
        ThrowPretty("Robot model may not be a quadrotor.");
}

Eigen::VectorXd QuadrotorDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    double phi = x(3),
           theta = x(4),
           psi = x(5),
           x_dot = x(6),
           y_dot = x(7),
           z_dot = x(8),
           phi_dot = x(9),
           theta_dot = x(10),
           psi_dot = x(11);

    const double F_1 = k_f_ * u(0);
    const double F_2 = k_f_ * u(1);
    const double F_3 = k_f_ * u(2);
    const double F_4 = k_f_ * u(3);

    const double M_1 = k_m_ * u(0);
    const double M_2 = k_m_ * u(1);
    const double M_3 = k_m_ * u(2);
    const double M_4 = k_m_ * u(3);

    // clang-format off
    double sin_phi = std::sin(phi),     cos_phi = std::cos(phi),
           sin_theta = std::sin(theta), cos_theta = std::cos(theta),
           sin_psi = std::sin(psi),     cos_psi = std::cos(psi);
    // clang-format on

    Eigen::MatrixXd Rx(3, 3), Ry(3, 3), Rz(3, 3), R;
    Rx << 1, 0, 0, 0, cos_phi, -sin_phi, 0, sin_phi, cos_phi;
    Ry << cos_theta, 0, sin_theta, 0, 1, 0, -sin_theta, 0, cos_theta;
    Rz << cos_psi, -sin_psi, 0, sin_psi, cos_psi, 0, 0, 0, 1;
    R = Rz * Rx * Ry;

    // x,y,z dynamics
    Eigen::Vector3d Gtot, Ftot;
    Gtot << 0, 0, -g_;
    Ftot << 0, 0, F_1 + F_2 + F_3 + F_4;

    Eigen::Vector3d pos_ddot = Gtot + R * Ftot / mass_;

    // phi, theta, psi dynamics
    Eigen::Vector3d tau, omega, omega_dot;
    tau << L_ * (F_1 - F_2),
        L_ * (F_1 - F_3),
        (M_1 - M_2 + M_3 - M_4);
    omega << phi_dot, theta_dot, psi_dot;

    double radius = L_ / 2.0;
    double Ix = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_,
           Iy = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_,
           Iz = 2 * mass_ * (radius * radius) / 5.0 + 4 * radius * radius * mass_;

    Eigen::Matrix3d I, Iinv;
    I << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz;
    Iinv << 1 / Ix, 0, 0, 0, 1 / Iy, 0, 0, 0, 1 / Iz;

    omega_dot = Iinv * (tau - omega.cross(I * omega));

    Eigen::VectorXd state_dot(12);
    state_dot << x_dot, y_dot, z_dot,
        phi_dot, theta_dot, psi_dot,
        pos_ddot(0), pos_ddot(1), pos_ddot(2),
        omega_dot(0), omega_dot(1), omega_dot(2);

    return state_dot;
}

Eigen::MatrixXd QuadrotorDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    double phi = x(3),
           theta = x(4),
           psi = x(5),
           phi_dot = x(9),
           theta_dot = x(10),
           psi_dot = x(11);

    // clang-format off
    double sin_phi = std::sin(phi),     cos_phi = std::cos(phi),
           sin_theta = std::sin(theta), cos_theta = std::cos(theta),
           sin_psi = std::sin(psi),     cos_psi = std::cos(psi);

    Eigen::MatrixXd fx(num_positions_ + num_velocities_, num_positions_ + num_velocities_);
    fx <<
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, (k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))*sin_psi*cos_phi*cos_theta/mass_, (-sin_phi*sin_psi*sin_theta + cos_psi*cos_theta)*(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))/mass_, (sin_phi*cos_psi*cos_theta - sin_psi*sin_theta)*(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))/mass_, 0, 0, 0, 0, 0, 0,
        0, 0, 0, -(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))*cos_phi*cos_psi*cos_theta/mass_, (sin_phi*sin_theta*cos_psi + sin_psi*cos_theta)*(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))/mass_, (sin_phi*sin_psi*cos_theta + sin_theta*cos_psi)*(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))/mass_, 0, 0, 0, 0, 0, 0,
        0, 0, 0, -(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))*sin_phi*cos_theta/mass_, -(k_f_*u(0) + k_f_*u(1) + k_f_*u(2) + k_f_*u(3))*sin_theta*cos_phi/mass_, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.833333333333334*psi_dot, -0.833333333333334*theta_dot,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0.833333333333334*psi_dot, 0, 0.833333333333334*phi_dot,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // clang-format on
    return fx;
}

Eigen::MatrixXd QuadrotorDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    double phi = x(3),
           theta = x(4),
           psi = x(5);

    // clang-format off
    double sin_phi = std::sin(phi),     cos_phi = std::cos(phi),
           sin_theta = std::sin(theta), cos_theta = std::cos(theta),
           sin_psi = std::sin(psi),     cos_psi = std::cos(psi);

    Eigen::MatrixXd fu(num_positions_ + num_velocities_, num_controls_);
    fu << 
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        k_f_*(sin_phi*sin_psi*cos_theta + sin_theta*cos_psi)/mass_, k_f_*(sin_phi*sin_psi*cos_theta + sin_theta*cos_psi)/mass_, k_f_*(sin_phi*sin_psi*cos_theta + sin_theta*cos_psi)/mass_, k_f_*(sin_phi*sin_psi*cos_theta + sin_theta*cos_psi)/mass_,
        k_f_*(-sin_phi*cos_psi*cos_theta + sin_psi*sin_theta)/mass_, k_f_*(-sin_phi*cos_psi*cos_theta + sin_psi*sin_theta)/mass_, k_f_*(-sin_phi*cos_psi*cos_theta + sin_psi*sin_theta)/mass_, k_f_*(-sin_phi*cos_psi*cos_theta + sin_psi*sin_theta)/mass_,
        k_f_*cos_phi*cos_theta/mass_, k_f_*cos_phi*cos_theta/mass_, k_f_*cos_phi*cos_theta/mass_, k_f_*cos_phi*cos_theta/mass_,
        1.66666666666667*k_f_/(L_*mass_), -1.66666666666667*k_f_/(L_*mass_), 0, 0,
        1.66666666666667*k_f_/(L_*mass_), 0, -1.66666666666667*k_f_/(L_*mass_), 0,
        0.909090909090909*k_m_/(L_*L_*2*mass_), -0.909090909090909*k_m_/(L_*L_*2*mass_), 0.909090909090909*k_m_/(L_*L_*mass_), -0.909090909090909*k_m_/(L_*L_*mass_);

    // clang-format on
    return fu;
}

}  // namespace exotica
