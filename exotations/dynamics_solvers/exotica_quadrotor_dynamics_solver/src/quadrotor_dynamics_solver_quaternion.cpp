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

// NOTE: Unused
#include <exotica_quadrotor_dynamics_solver/quadrotor_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("QuadrotorDynamicsSolver", exotica::QuadrotorDynamicsSolver)

namespace exotica
{
QuadrotorDynamicsSolver::QuadrotorDynamicsSolver()
{
    num_positions_ = 7;
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
    Eigen::Quaterniond quaternion;

    // If input quaternion is (0,0,0,0) set to (1,0,0,0)
    if (x.segment<4>(3).isApprox(Eigen::Vector4d::Zero()))
        quaternion = Eigen::Quaterniond(1, 0, 0, 0);
    else
        quaternion = Eigen::Quaterniond(x.segment<4>(3)).normalized();

    const Eigen::Vector3d v = x.segment<3>(6);
    const Eigen::Vector3d omega = x.tail<3>();

    const double F_1 = k_f_ * u(0);
    const double F_2 = k_f_ * u(1);
    const double F_3 = k_f_ * u(2);
    const double F_4 = k_f_ * u(3);
    const Eigen::Vector3d F(0, 0, F_1 + F_2 + F_3 + F_4);  // total rotor force in body frame

    const double M_1 = k_m_ * u(0);
    const double M_2 = k_m_ * u(1);
    const double M_3 = k_m_ * u(2);
    const double M_4 = k_m_ * u(3);
    const Eigen::Vector3d tau(L_ * (F_2 - F_4), L_ * (F_3 - F_1), (M_1 - M_2 + M_3 - M_4));  // total rotor torque in body frame

    StateVector x_dot(13);
    x_dot.head<3>() = v;                                                                                      // velocity in world frame
    x_dot.segment<4>(3) = 0.5 * (quaternion * Eigen::Quaterniond(0, omega(0), omega(1), omega(2))).coeffs();  // via quaternion derivative (cf. https://math.stackexchange.com/a/2099673)
    x_dot.segment<3>(7) = Eigen::Vector3d(0, 0, -g_) + (1. / mass_) * (quaternion * F);                       // acceleration in world frame
    x_dot.segment<3>(10) = J_inv_ * (tau - omega.cross(J_ * omega));                                          // Euler's equation : I*ω + ω x I*ω

    return x_dot;
}

Eigen::MatrixXd QuadrotorDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    // TODO
    ThrowPretty("NotYetImplemented");
}

Eigen::MatrixXd QuadrotorDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    // TODO
    ThrowPretty("NotYetImplemented");
}

Eigen::VectorXd QuadrotorDynamicsSolver::GetPosition(Eigen::VectorXdRefConst x_in)
{
    // Convert quaternion to Euler angles.
    Eigen::Matrix<double, 6, 1> xyz_rpy;
    xyz_rpy.head<3>() = x_in.head<3>();
    xyz_rpy.tail<3>() = Eigen::Quaterniond(x_in.segment<4>(3)).toRotationMatrix().eulerAngles(0, 1, 2);
    return xyz_rpy;
}
}  // namespace exotica
