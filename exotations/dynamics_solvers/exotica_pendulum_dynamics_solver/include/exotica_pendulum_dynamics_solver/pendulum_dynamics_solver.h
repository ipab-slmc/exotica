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

#ifndef EXOTICA_PENDULUM_DYNAMICS_SOLVER_PENDULUM_DYNAMICS_SOLVER_H_
#define EXOTICA_PENDULUM_DYNAMICS_SOLVER_PENDULUM_DYNAMICS_SOLVER_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_pendulum_dynamics_solver/pendulum_dynamics_solver_initializer.h>

namespace exotica
{
/// Refer to http://underactuated.mit.edu/underactuated.html?chapter=pend
///     for a derivation of the pendulum dynamics.
class PendulumDynamicsSolver : public DynamicsSolver, public Instantiable<PendulumDynamicsSolverInitializer>
{
public:
    PendulumDynamicsSolver();
    void AssignScene(ScenePtr scene_in) override;

    /// \brief Computes the forward dynamics of the system.
    /// @param x The state vector.
    /// @param u The control input.
    /// @return The dynamics transition function.
    StateVector f(const StateVector& x, const ControlVector& u) override;

    /// \brief Computes the dynamics derivative w.r.t .the state x.
    /// @param x The state vector.
    /// @param u The control input.
    /// @return The derivative of the dynamics function w.r.t. x evaluated at (x, u).
    Eigen::MatrixXd fx(const StateVector& x, const ControlVector& u) override;
    /// \brief Computes the dynamics derivative w.r.t .the control input u.
    /// @param x The state vector.
    /// @param u The control input
    /// @return The derivative of the dynamics function w.r.t. u evaluated at (x, u).
    Eigen::MatrixXd fu(const StateVector& x, const ControlVector& u) override;

private:
    double g_ = 9.81;  ///!< Gravity (m/s^2)
    double m_ = 1.0;   ///!< Mass at the end of the pendulum
    double l_ = 1.0;   ///!< Length of the arm
    double b_ = 0.0;   ///!< Friction coefficient
};
}  // namespace exotica

#endif  // EXOTICA_PENDULUM_DYNAMICS_SOLVER_PENDULUM_DYNAMICS_SOLVER_H_
