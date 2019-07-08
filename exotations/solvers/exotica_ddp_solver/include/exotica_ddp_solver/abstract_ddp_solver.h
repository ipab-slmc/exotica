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
#include <exotica_core/tools/conversions.h>

namespace exotica
{
// \brief Base DDP Solver class that implements the forward pass.
//  and utility functions. This is a templated class since we need
//  the parameters from the solvers but do not have parameters here.
//
//  This is why this is a header-only
template <typename Initializer>
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

    Initializer base_parameters_;
    Eigen::MatrixXd best_ref_x_, best_ref_u_;  ///!< Reference trajectory for feedback control.

    double lambda_ = 0.1,
           lambda_max_ = 1000;  ///!< Levenberg Marquardt parameters
};

template <typename Initializer>
void AbstractDDPSolver<Initializer>::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    const int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();

    // TODO: parametrize
    lambda_ = 0.1;

    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    // initialize Gain matrices
    K_gains_.assign(T, Eigen::MatrixXd(NU, NX));
    k_gains_.assign(T, Eigen::VectorXd(NU, 1));

    // all of the below are not pointers, since we want to copy over
    //  solutions across iterations
    Eigen::MatrixXd new_U, global_best_U;
    solution.resize(T, NU);

    if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Running DDP solver for max " << GetNumberOfMaxIterations() << " iterations");

    double last_cost = 0, global_best_cost = 0;
    int last_best_iteration = 0;

    for (int iteration = 0; iteration < GetNumberOfMaxIterations(); ++iteration)
    {
        // Backwards pass computes the gains
        backward_pass_timer.Reset();
        BackwardPass();
        if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Backward pass complete in " << backward_pass_timer.GetDuration() << " lambda=" << lambda_);

        line_search_timer.Reset();
        // forward pass to compute new control trajectory
        // TODO: Configure line-search space from xml
        // TODO (Wolf): What you are doing is a forward line-search when we may try a
        //    backtracking line search for a performance improvement later - this just as an aside.
        const Eigen::VectorXd alpha_space = Eigen::VectorXd::LinSpaced(10, 0.1, 1.0);
        double current_cost = 0, best_alpha = 0;

        Eigen::MatrixXd ref_x = prob_->get_X(),
                        ref_u = prob_->get_U();
        // perform a linear search to find the best rate
        for (int ai = 0; ai < alpha_space.rows(); ++ai)
        {
            double alpha = alpha_space(ai);
            double cost = ForwardPass(alpha, ref_x, ref_u);

            if (ai == 0 || (cost < current_cost && std::isfinite(cost)))
            {
                current_cost = cost;
                new_U = prob_->get_U();
                best_alpha = alpha;
            }
        }
        
        if (!std::isfinite(current_cost))
        {
            if (debug_) HIGHLIGHT_NAMED("DDOSolver", "Diverged!");
            break;
        }

        // source: https://uk.mathworks.com/help/optim/ug/least-squares-model-fitting-algorithms.html, eq. 13
        if (iteration > 0)
        {
            if (current_cost < last_cost)
            {
                // success, error decreased: decrease damping
                lambda_ = lambda_ / 10.0;
            }
            else
            {
                // failure, error increased: increase damping
                lambda_ = lambda_ * 10.0;
            }
        }

        if (lambda_ > lambda_max_)
        {
            HIGHLIGHT_NAMED("DDPSolver", "Lambda greater than maximum.");
            break;
        }

        if (debug_)
        {
            HIGHLIGHT_NAMED("DDPSolver", "Forward pass complete in " << line_search_timer.GetDuration() << " with cost: " << current_cost << " and alpha " << best_alpha);
            HIGHLIGHT_NAMED("DDPSolver", "Final state: " << prob_->get_X(T - 1).transpose());
        }

        // copy solutions for next iteration
        if (iteration == 0 || global_best_cost > current_cost)
        {
            global_best_cost = current_cost;
            last_best_iteration = iteration;
            global_best_U = new_U;
            best_ref_x_ = ref_x;
            best_ref_u_ = ref_u;
        }

        if (iteration - last_best_iteration > base_parameters_.FunctionTolerancePatience)
        {
            if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Early stopping criterion reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (last_cost - current_cost < base_parameters_.FunctionTolerance && last_cost - current_cost > 0)
        {
            if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Function tolerance reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (debug_ && iteration == GetNumberOfMaxIterations() - 1)
            HIGHLIGHT_NAMED("DDPSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());

        last_cost = current_cost;
        for (int t = 0; t < T - 1; ++t)
            prob_->Update(new_U.col(t), t);
        prob_->SetCostEvolution(iteration, current_cost);
    }

    // store the best solution found over all iterations
    for (int t = 0; t < T - 1; ++t)
    {
        solution.row(t) = global_best_U.col(t).transpose();
        prob_->Update(global_best_U.col(t), t);
    }

    planning_time_ = planning_timer.GetDuration();
}

template <typename Initializer>
void AbstractDDPSolver<Initializer>::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This DDPSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();
    if (debug_) HIGHLIGHT_NAMED("DDPSolver", "initialized");
}

template <typename Initializer>
double AbstractDDPSolver<Initializer>::ForwardPass(const double alpha, Eigen::MatrixXdRefConst ref_x, Eigen::MatrixXdRefConst ref_u)
{
    double cost = 0;
    const int T = prob_->get_T();
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    const double dt = dynamics_solver_->get_dt();

    for (int t = 0; t < T - 1; ++t)
    {
        Eigen::VectorXd u = ref_u.col(t);
        // eq. 12
        Eigen::VectorXd delta_uk = k_gains_[t] + K_gains_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), ref_x.col(t));
        u.noalias() += alpha * delta_uk;
        // clamp controls
        u = u.cwiseMax(-control_limits).cwiseMin(control_limits);

        prob_->Update(u, t);
        cost += dt * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // add terminal cost
    cost += prob_->GetStateCost(T - 1);
    return cost;
}

template <typename Initializer>
Eigen::VectorXd AbstractDDPSolver<Initializer>::GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const
{
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    Eigen::VectorXd delta_uk = k_gains_[t] + K_gains_[t] * dynamics_solver_->StateDelta(x, best_ref_x_.col(t));

    Eigen::VectorXd u = best_ref_u_.col(t) + delta_uk;
    return u.cwiseMax(-control_limits).cwiseMin(control_limits);
}

}  // namespace exotica

#endif  // EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_
