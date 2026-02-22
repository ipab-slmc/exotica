//
// Copyright (c) 2019, University of Edinburgh
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

#ifndef EXOTICA_ILQR_SOLVER_ILQR_SOLVER_H_
#define EXOTICA_ILQR_SOLVER_ILQR_SOLVER_H_

#include <exotica_core/feedback_motion_solver.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
#include <exotica_core/server.h>

#include <exotica_ilqr_solver/ilqr_solver_initializer.h>

namespace exotica
{
// This code is based on:
// W. Li, E. Todorov, Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems
// https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf
class ILQRSolver : public FeedbackMotionSolver, public Instantiable<ILQRSolverInitializer>
{
public:
    ///\brief Solves the problem
    ///@param solution Returned solution trajectory as a vector of joint configurations.
    void Solve(Eigen::MatrixXd& solution) override;

    ///\brief Binds the solver to a specific problem which must be pre-initalised
    ///@param pointer Shared pointer to the motion planning problem
    ///@return        Successful if the problem is a valid DynamicTimeIndexedProblem
    void SpecifyProblem(PlanningProblemPtr pointer) override;

    Eigen::VectorXd GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const override;

private:
    DynamicTimeIndexedShootingProblemPtr prob_;                              ///!< Shared pointer to the planning problem.
    DynamicsSolverPtr dynamics_solver_;                                      ///!< Shared pointer to the dynamics solver.
    std::vector<Eigen::MatrixXd> K_gains_, Ku_gains_, Kv_gains_, vk_gains_;  ///!< Control gains.

    Eigen::MatrixXd best_ref_x_, best_ref_u_;  ///!< Reference trajectory for feedback control.

    ///\brief Computes the control gains for a the trajectory in the associated
    ///     DynamicTimeIndexedProblem.
    void BackwardPass();

    ///\brief Forward simulates the dynamics using the gains computed in the
    ///     last BackwardPass;
    /// @param alpha The learning rate.
    /// @param ref_trajectory The reference state trajectory.
    /// @return The cost associated with the new control and state trajectory.
    double ForwardPass(const double alpha, Eigen::MatrixXdRefConst ref_x, Eigen::MatrixXdRefConst ref_u);
};
}  // namespace exotica

#endif  // EXOTICA_ILQR_SOLVER_ILQR_SOLVER_H_
