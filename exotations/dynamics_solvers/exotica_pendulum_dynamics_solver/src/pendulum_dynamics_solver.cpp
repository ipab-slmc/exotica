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

#include <exotica_pendulum_dynamics_solver/pendulum_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("PendulumDynamicsSolver", exotica::PendulumDynamicsSolver)

namespace exotica
{
PendulumDynamicsSolver::PendulumDynamicsSolver()
{
    num_positions_ = 1;
    num_velocities_ = 1;
    num_controls_ = 1;
}

void PendulumDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const int num_positions_in = scene_in->GetKinematicTree().GetNumControlledJoints();
    // TODO: This is a terrible check (not against name etc.), but can stop _some_ mismatches between URDF/model and dynamics
    if (num_positions_in != 1)
        ThrowPretty("Robot model may not be a Pendulum.");

    if (parameters_.FrictionCoefficient < 0)
        ThrowPretty("Coefficient of friction is less than 0 (" << parameters_.FrictionCoefficient << ").");
    if (parameters_.FrictionCoefficient > 1.5)
        WARNING_NAMED("PendulumDynamicsSolver", "Coefficient of friction " << parameters_.FrictionCoefficient << " might be too high!");
    b_ = parameters_.FrictionCoefficient;
}

Eigen::VectorXd PendulumDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    auto theta = x(0);
    auto thetadot = x(1);

    auto x_dot = StateVector(2);
    x_dot << thetadot,
        (u(0) - m_ * g_ * l_ * std::sin(theta) - b_ * thetadot) / (m_ * l_ * l_);

    return x_dot;
}

// NOTE: tested in test/test_pendulum_diff.py in this package
Eigen::MatrixXd PendulumDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    auto theta = x(0);

    Eigen::Matrix2d fx;
    fx << 0, 1,
        -g_ * std::cos(theta) / l_, -b_ / (l_ * l_ * m_);

    return fx;
}

// NOTE: tested in test/test_pendulum_diff.py in this package
Eigen::MatrixXd PendulumDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    Eigen::Vector2d fu;
    fu << 0, 1.0 / (l_ * l_ * m_);
    return fu;
}
}  // namespace exotica
