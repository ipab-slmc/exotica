//
// Copyright (c) 2019, Traiko Dinev
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

#include <exotica_cartpole_dynamics_solver/cartpole_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("CartpoleDynamicsSolver", exotica::CartpoleDynamicsSolver)

namespace exotica
{
CartpoleDynamicsSolver::CartpoleDynamicsSolver()
{
    num_positions_ = 2;
    num_velocities_ = 2;
    num_controls_ = 1;
    has_second_order_derivatives_ = true;
}

void CartpoleDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const int num_positions_in = scene_in->GetKinematicTree().GetNumControlledJoints();
    // TODO: This is a terrible check (not against name etc.), but can stop _some_ mismatches between URDF/model and dynamics
    if (num_positions_in != 2)
        ThrowPretty("Robot model may not be a Cartpole.");
}

Eigen::VectorXd CartpoleDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    const double& theta = x(1);
    const double& xdot = x(2);
    const double& thetadot = x(3);

    auto sin_theta = std::sin(theta);
    auto cos_theta = std::cos(theta);
    auto theta_dot_squared = thetadot * thetadot;

    auto x_dot = StateVector(4);
    x_dot << xdot, thetadot,
        (u(0) + m_p_ * sin_theta * (l_ * theta_dot_squared + g_ * cos_theta)) /
            (m_c_ + m_p_ * sin_theta * sin_theta),
        -(l_ * m_p_ * cos_theta * sin_theta * theta_dot_squared + u(0) * cos_theta +
          (m_c_ + m_p_) * g_ * sin_theta) /
            (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta);

    return x_dot;
}

// NOTE: tested in test/test_cartpole_diff.py in this package
Eigen::MatrixXd CartpoleDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    const double& theta = x(1);
    const double& tdot = x(3);

    auto sin_theta = std::sin(theta);
    auto cos_theta = std::cos(theta);

    Eigen::Matrix4d fx;
    fx << 0, 0, 1, 0,
        0, 0, 0, 1,
        //
        0,
        -2 * m_p_ * (m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * sin_theta + u(0)) * sin_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) + (-g_ * m_p_ * sin_theta * sin_theta + m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * cos_theta) / (m_c_ + m_p_ * sin_theta * sin_theta),
        0,
        2 * l_ * m_p_ * tdot * sin_theta / (m_c_ + m_p_ * sin_theta * sin_theta),
        //
        0,
        -2 * l_ * m_p_ * (-g_ * (m_c_ + m_p_) * sin_theta - l_ * m_p_ * tdot * tdot * sin_theta * cos_theta - u(0) * cos_theta) * sin_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) + (-g_ * (m_c_ + m_p_) * cos_theta + l_ * m_p_ * tdot * tdot * sin_theta * sin_theta - l_ * m_p_ * tdot * tdot * cos_theta * cos_theta + u(0) * sin_theta) / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta),
        0,
        -2 * l_ * m_p_ * tdot * sin_theta * cos_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta);

    return fx;
}

// NOTE: tested in test/test_cartpole_diff.py in this package
Eigen::MatrixXd CartpoleDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    const double& theta = x(1);

    auto sin_theta = std::sin(theta);
    auto cos_theta = std::cos(theta);

    Eigen::Vector4d fu;
    fu << 0, 0, 1 / (m_c_ + m_p_ * sin_theta * sin_theta), -cos_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta);
    return fu;
}

// NOTE: Code to generate 2nd order dynamics is in scripts/gen_second_order_dynamics.py
// fxx -> NX x NX x NX
//  middle dimension is always NX
// NX = 4
// NU = 1
Eigen::Tensor<double, 3> CartpoleDynamicsSolver::fxx(const StateVector& x, const ControlVector& u)
{
    const double& theta = x(1);
    const double& tdot = x(3);

    auto sin_theta = std::sin(theta);
    auto cos_theta = std::cos(theta);

    Eigen::Tensor<double, 3> fxx(num_positions_ + num_velocities_, num_positions_ + num_velocities_, num_positions_ + num_velocities_);
    fxx.setValues({{{0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0}},
                   {{0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0,
                     8 * m_p_ * m_p_ * (m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * sin_theta + u(0)) * sin_theta * sin_theta * cos_theta * cos_theta /
                             std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 3) -
                         4 * m_p_ * (-g_ * m_p_ * sin_theta * sin_theta + m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * cos_theta) * sin_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) +
                         2 * m_p_ * (m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * sin_theta + u(0)) * sin_theta * sin_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) - 2 * m_p_ * (m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * sin_theta + u(0)) * cos_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) + (-3 * g_ * m_p_ * sin_theta * cos_theta - m_p_ * (g_ * cos_theta + l_ * tdot * tdot) * sin_theta) / (m_c_ + m_p_ * sin_theta * sin_theta),
                     0,
                     -4 * l_ * m_p_ * m_p_ * tdot * sin_theta * sin_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) + 2 * l_ * m_p_ * tdot * cos_theta / (m_c_ + m_p_ * sin_theta * sin_theta)},
                    {0,
                     8 * l_ * l_ * m_p_ * m_p_ * (-g_ * (m_c_ + m_p_) * sin_theta - l_ * m_p_ * tdot * tdot * sin_theta * cos_theta - u(0) * cos_theta) * sin_theta * sin_theta * cos_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 3) + 2 * l_ * m_p_ * (-g_ * (m_c_ + m_p_) * sin_theta - l_ * m_p_ * tdot * tdot * sin_theta * cos_theta - u(0) * cos_theta) * sin_theta * sin_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) - 2 * l_ * m_p_ * (-g_ * (m_c_ + m_p_) * sin_theta - l_ * m_p_ * tdot * tdot * sin_theta * cos_theta - u(0) * cos_theta) * cos_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) - 4 * l_ * m_p_ * (-g_ * (m_c_ + m_p_) * cos_theta + l_ * m_p_ * tdot * tdot * sin_theta * sin_theta - l_ * m_p_ * tdot * tdot * cos_theta * cos_theta + u(0) * sin_theta) * sin_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) + (g_ * (m_c_ + m_p_) * sin_theta + 4 * l_ * m_p_ * tdot * tdot * sin_theta * cos_theta + u(0) * cos_theta) / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta),
                     0,
                     4 * l_ * l_ * m_p_ * m_p_ * tdot * sin_theta * sin_theta * cos_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) + 2 * l_ * m_p_ * tdot * sin_theta * sin_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta) - 2 * l_ * m_p_ * tdot * cos_theta * cos_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta)}},
                   {{0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0}},
                   {{0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0,
                     -4 * l_ * m_p_ * m_p_ * tdot * sin_theta * sin_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2) + 2 * l_ * m_p_ * tdot * cos_theta / (m_c_ + m_p_ * sin_theta * sin_theta),
                     0, 2 * l_ * m_p_ * sin_theta / (m_c_ + m_p_ * sin_theta * sin_theta)},
                    {0,
                     4 * l_ * l_ * m_p_ * m_p_ * tdot * sin_theta * sin_theta * cos_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) +
                         (2 * l_ * m_p_ * tdot * sin_theta * sin_theta - 2 * l_ * m_p_ * tdot * cos_theta * cos_theta) / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta),
                     0, -2 * l_ * m_p_ * sin_theta * cos_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta)}}});
    return fxx;
}

Eigen::Tensor<double, 3> CartpoleDynamicsSolver::fxu(const StateVector& x, const ControlVector& u)
{
    const double& theta = x(1);

    auto sin_theta = std::sin(theta);
    auto cos_theta = std::cos(theta);

    Eigen::Tensor<double, 3> fxu(num_positions_ + num_velocities_, num_positions_ + num_velocities_, num_controls_);
    fxu.setValues({{{0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, -2 * m_p_ * sin_theta * cos_theta / std::pow(m_c_ + m_p_ * sin_theta * sin_theta, 2), 0, 0},
                    {0, 2 * l_ * m_p_ * sin_theta * cos_theta * cos_theta / std::pow(l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta, 2) + sin_theta / (l_ * m_c_ + l_ * m_p_ * sin_theta * sin_theta),
                     0, 0}}});
    return fxu;
}

}  // namespace exotica