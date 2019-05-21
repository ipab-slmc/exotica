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

#ifndef EXOTICA_CARTPOLE_DYNAMICS_SOLVER_CARTPOLE_DYNAMICS_SOLVER_H_
#define EXOTICA_CARTPOLE_DYNAMICS_SOLVER_CARTPOLE_DYNAMICS_SOLVER_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_cartpole_dynamics_solver/cartpole_dynamics_solver_initializer.h>

namespace exotica
{
/// StateVector X ∈ R^4 = [x, theta, x_dot, theta_dot]
class CartpoleDynamicsSolver : public DynamicsSolver, public Instantiable<CartpoleDynamicsSolverInitializer>
{
public:
    CartpoleDynamicsSolver();
    void AssignScene(ScenePtr scene_in) override;

    /// \brief Computes the forward dynamics of the system.
    /// @param x The state vector.
    /// @param u The control input.
    /// @returns The dynamics transition function.
    StateVector f(const StateVector& x, const ControlVector& u) override;

    /// \brief Computes the dynamics derivative w.r.t .the state x.
    /// @param x The state vector.
    /// @param u The control input.
    /// @returns The derivative of the dynamics function w.r.t. x evaluated at (x, u).
    Eigen::MatrixXd fx(const StateVector& x, const ControlVector& u) override;
    /// \brief Computes the dynamics derivative w.r.t .the control input u.
    /// @param x The state vector.
    /// @param u The control input
    /// @returns The derivative of the dynamics function w.r.t. u evaluated at (x, u).
    Eigen::MatrixXd fu(const StateVector& x, const ControlVector& u) override;

    /// \brief
    /// Reeturns the position of all the joints. Converts to the coordinates used by the model.
    /// @param x The state vector.
    /// @returns State vector in model coordinates.
    Eigen::VectorXd GetPosition(Eigen::VectorXdRefConst x_in) override;

    /// \brief
    /// Reeturns the difference between two state vectors x_1 - x_2.
    Eigen::VectorXd StateDelta(const StateVector& x_1, const StateVector& x_2) override;

private:
    Eigen::Matrix3d M;      ///!< Inertia (mass) matrix
    Eigen::Matrix3d M_inv;  ///!< Inverted inertia matrix

    double g_ = 9.81;                         ///!< Gravity (m/s^2)
    double m_c = 1;                           ///!< Cart mass (kg)
    double m_p = 1;                           ///!< Pole mass (kg)
    double l = 1;                             ///!< Pole length (kg)
    double slider_length = 30;                ///!< Length of slider (m).
    double max_x = this->slider_length / 2;   ///!< Left end of the slider
    double max_y = -this->slider_length / 2;  ///!< Right end of the slider
};
}  // namespace exotica

#endif  // EXOTICA_CARTPOLE_DYNAMICS_SOLVER_CARTPOLE_DYNAMICS_SOLVER_H_
