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

#ifndef EXOTICA_QUADROTOR_DYNAMICS_SOLVER_QUADROTOR_DYNAMICS_SOLVER_H_
#define EXOTICA_QUADROTOR_DYNAMICS_SOLVER_QUADROTOR_DYNAMICS_SOLVER_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_quadrotor_dynamics_solver/quadrotor_dynamics_solver_initializer.h>

namespace exotica
{
/// \brief Quadrotor dynamics with quaternion representation
/// Based on D. Mellinger, N. Michael, and V. Kumar,
/// "Trajectory generation and control for precise aggressive maneuvers with quadrotors",
/// Proceedings of the 12th International Symposium on Experimental Robotics (ISER 2010), 2010.
/// Cf. https://journals.sagepub.com/doi/abs/10.1177/0278364911434236
///
/// StateVector X ∈ R^12 = [x, y, z, r, p, y, xdot, ydot, zdot, omega1, omega2, omega3]
class QuadrotorDynamicsSolver : public DynamicsSolver, public Instantiable<QuadrotorDynamicsSolverInitializer>
{
public:
    QuadrotorDynamicsSolver();

    void AssignScene(ScenePtr scene_in) override;

    StateVector f(const StateVector& x, const ControlVector& u) override;
    Eigen::MatrixXd fx(const StateVector& x, const ControlVector& u) override;
    Eigen::MatrixXd fu(const StateVector& x, const ControlVector& u) override;

    // Eigen::VectorXd GetPosition(Eigen::VectorXdRefConst x_in) override;

private:
    Eigen::Matrix3d J_;      ///< Inertia matrix
    Eigen::Matrix3d J_inv_;  ///< Inverted inertia matrix

    double mass_ = .5;   ///< Mass
    double g_ = 9.81;    ///< Gravity (m/s^2)
    double L_ = 0.1750;  ///< Distance between motors

    // these are measured in rpm * 10^3
    //  this means we multiply here by 10^6 (since it varies with omega^2)
    //
    // Additionally, we assume we control the thrust directly,
    //  which leads to the following simplification:
    double k_f_ = 1;       ///< Thrust coefficient, 6.11*10^-8;
    double k_m_ = 0.0245;  ///< Moment coefficient

    // double b_ = 0.0245;  ///< Drag
};
}  // namespace exotica

#endif  // EXOTICA_QUADROTOR_DYNAMICS_SOLVER_QUADROTOR_DYNAMICS_SOLVER_H_
