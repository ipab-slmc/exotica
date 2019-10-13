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

#ifndef EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_
#define EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_

#include <exotica_core/feedback_motion_solver.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
#include <exotica_core/server.h>
#include <exotica_core/tools/conversions.h>
#include <exotica_ddp_solver/abstract_ddp_solver_initializer.h>

namespace exotica
{
// \brief Base DDP Solver class that implements the forward pass.
//  and utility functions.
class AbstractDDPSolver : public FeedbackMotionSolver
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

protected:
    DynamicTimeIndexedShootingProblemPtr prob_;       ///!< Shared pointer to the planning problem.
    DynamicsSolverPtr dynamics_solver_;               ///!< Shared pointer to the dynamics solver.
    std::vector<Eigen::MatrixXd> K_gains_, k_gains_;  ///!< Control gains.

    ///\brief Computes the control gains for a the trajectory in the associated
    ///     DynamicTimeIndexedProblem.
    virtual void BackwardPass() = 0;

    ///\brief Forward simulates the dynamics using the gains computed in the
    ///     last BackwardPass;
    /// @param alpha The learning rate.
    /// @param ref_trajectory The reference state trajectory.
    /// @return The cost associated with the new control and state trajectory.
    double ForwardPass(const double alpha, Eigen::MatrixXdRefConst ref_x, Eigen::MatrixXdRefConst ref_u);

    AbstractDDPSolverInitializer base_parameters_;

    inline void IncreaseRegularization()
    {
        lambda_ *= 10.;
    }

    inline void DecreaseRegularization()
    {
        lambda_ /= 10.;
    }

    // Local variables used in the solver - copies get updated at the beginning of solve:
    Eigen::VectorXd alpha_space_;
    double lambda_;  ///!< Regularisation (Vxx, Quu)
    int T_;
    int NU_;
    int NX_;
    double dt_;
    double cost_;        ///!< Cost during iteration
    double cost_prev_;   ///!< Cost during previous iteration
    double alpha_best_;  ///!< Line-search step taken
    double time_taken_forward_pass_, time_taken_backward_pass_;
    Eigen::MatrixXd U_try_;   ///!< Updated control trajectory during iteration.
    Eigen::MatrixXd U_prev_;  ///!< Last accepted control trajectory
    Eigen::MatrixXd X_ref_;   ///!< Reference state trajectory for feedback control.
    Eigen::MatrixXd U_ref_;   ///!< Reference control trajectory for feedback control.
    Eigen::MatrixXd Qx_, Qu_, Qxx_, Quu_, Qux_, Quu_inv_, Vxx_;
    Eigen::VectorXd Vx_;
    Eigen::MatrixXd fx_, fu_;
};

}  // namespace exotica

#endif  // EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_
