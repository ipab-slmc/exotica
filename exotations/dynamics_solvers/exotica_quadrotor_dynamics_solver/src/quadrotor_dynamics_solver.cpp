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
    double x_ = x(0),
        y_ = x(1),
        z_ = x(2),
        phi = x(3),
        theta = x(4),
        psi = x(5),
        x_dot = x(6),
        y_dot = x(7),
        z_dot = x(8),
        phi_dot = x(9),
        theta_dot = x(10),
        psi_dot = x(11);

    double sin_phi = std::sin(phi),     cos_phi = std::cos(phi),        tan_phi = std::tan(phi),
        sin_theta = std::sin(theta), cos_theta = std::cos(theta),    tan_theta = std::tan(theta),
        sin_psi = std::sin(psi),     cos_psi = std::cos(psi),        tan_psi = std::tan(psi);

    Eigen::MatrixXd Rx(3, 3), Ry(3, 3), Rz(3, 3), R;
    Rx << 1, 0, 0, 0, cos_phi, -sin_phi, 0, sin_phi, cos_phi;
    Ry << cos_theta, 0, sin_theta, 0, 1, 0, - sin_theta, 0, cos_theta;
    Rz << cos_psi, -sin_psi, 0, sin_psi, cos_psi, 0, 0, 0, 1;
    R = Rz * Ry * Rx;

    // x,y,z dynamics
    Eigen::Vector3d Gtot, Ftot;
    Gtot << 0, 0, - mass_ * g_;
    Ftot << 0, 0, u(0) + u(1) + u(2) + u(3);
    
    Eigen::Vector3d pos_ddot = Gtot + R * Ftot / mass_;

    // drag
    constexpr double b = 0.0245;

    // phi, theta, psi dynamics
    Eigen::Vector3d tau, omega, omega_dot;
    tau << L_ * (u(0) - u(2)),
        L_ * (u(1) - u(3)),
        b * (u(0) - u(1) + u(2) - u(4));
    omega << phi_dot, theta_dot, psi_dot;

    double radius = L_ / 2.0;
    double Ix = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_,
        Iy = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_,
        Iz = 2 * mass_ * (radius * radius) / 5.0 + 4 * radius * radius * mass_;
    
    Eigen::Matrix3d I, Iinv;
    I << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz;
    Iinv << 1/Ix, 0, 0, 0, 1/Iy, 0, 0, 0, 1/Iz;

    omega_dot = Iinv * (
        tau - omega.cross(I * omega)
    );

    // HIGHLIGHT("Eat shit");

    Eigen::VectorXd state_dot(12);
    state_dot << x_dot, y_dot, z_dot,
        phi_dot, theta_dot, psi_dot,
        pos_ddot(0), pos_ddot(1), pos_ddot(2),
        omega_dot(0), omega_dot(1), omega_dot(2);
    
    return state_dot;
}

// Eigen::MatrixXd QuadrotorDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
// {
//     // Finite differences
//     constexpr double eps = 1e-6;
//     const int NX = num_positions_ + num_velocities_;

//     Eigen::MatrixXd fx_fd(NX, NX);

//     for (int i = 0; i < NX; ++i)
//     {
//         Eigen::VectorXd x_low = x;
//         Eigen::VectorXd x_high = x;
//         x_low(i) -= eps;
//         x_high(i) += eps;

//         fx_fd.col(i) = (f(x_high, u) - f(x_low, u)) / eps;
//     }
    
//     // HIGHLIGHT_NAMED("Pin", fx_fd);
//     return fx_fd;
// }

// Eigen::MatrixXd QuadrotorDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
// {
//     // Finite differences
//     constexpr double eps = 1e-6;
//     const int NX = num_positions_ + num_velocities_;
//     const int NU = num_controls_;

//     Eigen::MatrixXd fu_fd(NX, NU);

//     for (int i = 0; i < NU; ++i)
//     {
//         Eigen::VectorXd u_low = u;
//         Eigen::VectorXd u_high = u;
//         u_low(i) -= eps;
//         u_high(i) += eps;
        
//         fu_fd.col(i) = (f(x, u_high) - f(x, u_low)) / eps;
//     }

//     // HIGHLIGHT_NAMED("Pin", fu_fd);
//     return fu_fd;
// }

Eigen::VectorXd QuadrotorDynamicsSolver::GetPosition(Eigen::VectorXdRefConst x_in)
{
    // Convert quaternion to Euler angles.
    Eigen::Matrix<double, 6, 1> xyz_rpy;
    xyz_rpy.head<3>() = x_in.head<3>();
    xyz_rpy.tail<3>() = Eigen::Quaterniond(x_in.segment<4>(3)).toRotationMatrix().eulerAngles(0, 1, 2);
    return xyz_rpy;
}
}  // namespace exotica
