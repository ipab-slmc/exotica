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

#include <exotica_ddp_solver/abstract_ddp_solver.h>

namespace exotica
{
void AbstractDDPSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    T_ = prob_->get_T();
    NU_ = prob_->get_num_controls();
    NX_ = prob_->get_num_positions() + prob_->get_num_velocities();
    dt_ = dynamics_solver_->get_dt();
    lambda_ = base_parameters_.RegularizationRate;
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->PreUpdate();
    solution.resize(T_ - 1, NU_);

    // Perform initial roll-out
    cost_ = 0.0;
    for (int t = 0; t < T_ - 1; ++t)
    {
        prob_->Update(prob_->get_U(t), t);

        // Running cost
        cost_ += dt_ * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // Add terminal cost
    cost_ += prob_->GetStateCost(T_ - 1);
    prob_->SetCostEvolution(0, cost_);

    // Initialize gain matrices
    K_gains_.assign(T_, Eigen::MatrixXd(NU_, NX_));
    k_gains_.assign(T_, Eigen::VectorXd(NU_, 1));

    // Allocate memory by resizing commonly reused matrices:
    X_ref_.resize(NX_, T_);
    U_ref_.resize(NU_, T_ - 1);
    X_ref_ = prob_->get_X();
    U_ref_ = prob_->get_U();
    U_try_ = prob_->get_U();  // to resize/allocate
    // TODO: Resize the following:
    // Qx_, Qu_, Qxx_, Quu_, Qux_, Quu_inv_, Vxx_;
    // Vx_; fx_, fu_;

    if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Running DDP solver for max " << GetNumberOfMaxIterations() << " iterations");

    cost_prev_ = cost_;
    int last_best_iteration = 0;

    for (int iteration = 1; iteration <= GetNumberOfMaxIterations(); ++iteration)
    {
        // Check whether user interrupted (Ctrl+C)
        if (Server::IsRos() && !ros::ok())
        {
            if (debug_) HIGHLIGHT("Solving cancelled by user");
            prob_->termination_criterion = TerminationCriterion::UserDefined;
            break;
        }

        // Backward-pass computes the gains
        backward_pass_timer.Reset();
        BackwardPass();
        time_taken_backward_pass_ = backward_pass_timer.GetDuration();

        // Forward-pass to compute new control trajectory
        line_search_timer.Reset();

        double rollout_cost = cost_prev_;
        // Perform a linear search to find the best rate
        for (int ai = 0; ai < alpha_space_.size(); ++ai)
        {
            const double& alpha = alpha_space_(ai);
            rollout_cost = ForwardPass(alpha, X_ref_, U_ref_);

            if (rollout_cost < cost_)
            {
                cost_ = rollout_cost;
                U_try_ = prob_->get_U();
                alpha_best_ = alpha;
                break;
            }
        }
        time_taken_forward_pass_ = line_search_timer.GetDuration();

        // Finiteness checks
        if (!U_try_.allFinite())
        {
            prob_->termination_criterion = TerminationCriterion::Divergence;
            WARNING_NAMED("DDPSolver", "Divergence: Controls are non-finite");
            return;
        }
        if (!std::isfinite(cost_))
        {
            prob_->termination_criterion = TerminationCriterion::Divergence;
            WARNING_NAMED("DDPSolver", "Divergence: Cost is non-finite: " << cost_);
            return;
        }

        if (debug_)
        {
            HIGHLIGHT_NAMED("DDPSolver", "Iteration " << iteration << std::setprecision(3) << ":\tBackward pass: " << time_taken_backward_pass_ << " s\tForward pass: " << time_taken_forward_pass_ << " s\tCost: " << cost_ << "\talpha: " << alpha_best_ << "\tRegularization: " << lambda_);
        }

        //
        // Stopping criteria checks
        //

        // Relative function tolerance
        // (f_t-1 - f_t) <= functionTolerance * max(1, abs(f_t))
        if ((cost_prev_ - cost_) < base_parameters_.FunctionTolerance * std::max(1.0, std::abs(cost_)))
        {
            // Function tolerance patience check
            if (base_parameters_.FunctionTolerancePatience > 0)
            {
                if (iteration - last_best_iteration > base_parameters_.FunctionTolerancePatience)
                {
                    if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Early stopping criterion reached (" << cost_ << " < " << cost_prev_ << "). Time: " << planning_timer.GetDuration());
                    prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
                    break;
                }
            }
            else
            {
                if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Function tolerance reached (" << cost_ << " < " << cost_prev_ << "). Time: " << planning_timer.GetDuration());
                prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
                break;
            }
        }
        else
        {
            // Reset function tolerance patience
            last_best_iteration = iteration;
        }

        // Regularization
        if (lambda_ != 0.0 && lambda_ > 1e9)
        {
            prob_->termination_criterion = TerminationCriterion::Divergence;
            WARNING_NAMED("DDPSolver", "Divergence: Regularization too large (" << lambda_ << ")");
            return;
        }

        // If better than previous iteration, copy solutions for next iteration
        if (cost_ < cost_prev_)
        {
            cost_prev_ = cost_;
            U_ref_ = U_try_;

            if (alpha_best_ == alpha_space_(alpha_space_.size() - 1))
            {
                WARNING("Tiny step, increase!");
                IncreaseRegularization();
            }
            else
            {
                // if (debug_) HIGHLIGHT("Cost improved, decrease regularization");
                if (lambda_ > base_parameters_.MinimumRegularization) DecreaseRegularization();
            }
        }
        else
        {
            cost_ = cost_prev_;
            // Revert by not storing U_try_ as U_ref_ (maintain U_ref_)

            // if (debug_) HIGHLIGHT("No improvement - increase regularization");
            IncreaseRegularization();
        }

        // Roll-out and store reference state trajectory
        for (int t = 0; t < T_ - 1; ++t)
            prob_->Update(U_ref_.col(t), t);
        X_ref_ = prob_->get_X();

        prob_->SetCostEvolution(iteration, cost_);

        // Iteration limit
        if (iteration == GetNumberOfMaxIterations())
        {
            if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::IterationLimit;
        }
    }

    // Store the best solution found over all iterations
    for (int t = 0; t < T_ - 1; ++t)
    {
        solution.row(t) = U_ref_.col(t).transpose();
        prob_->Update(U_ref_.col(t), t);
    }

    planning_time_ = planning_timer.GetDuration();
}

void AbstractDDPSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This DDPSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();

    // Set up backtracking line-search coefficients
    alpha_space_ = Eigen::VectorXd::LinSpaced(11, 0.0, -3.0);
    for (int ai = 0; ai < alpha_space_.size(); ++ai)
    {
        alpha_space_(ai) = std::pow(10.0, alpha_space_(ai));
    }

    if (debug_) HIGHLIGHT_NAMED("DDPSolver", "initialized");
}

double AbstractDDPSolver::ForwardPass(const double alpha, Eigen::MatrixXdRefConst X_ref, Eigen::MatrixXdRefConst U_ref)
{
    double cost = 0.0;
    Eigen::VectorXd u_hat(NU_);  // TODO: allocate outside

    for (int t = 0; t < T_ - 1; ++t)
    {
        u_hat = U_ref.col(t);

        // eq. 12 - TODO: Which paper?
        u_hat.noalias() += alpha * k_gains_[t];
        u_hat.noalias() += K_gains_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), X_ref.col(t));

        // Clamp controls, if desired:
        if (base_parameters_.ClampControlsInForwardPass)
        {
            u_hat = u_hat.cwiseMax(dynamics_solver_->get_control_limits().col(0)).cwiseMin(dynamics_solver_->get_control_limits().col(1));
        }

        prob_->Update(u_hat, t);
        cost += dt_ * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // add terminal cost
    cost += prob_->GetStateCost(T_ - 1);
    return cost;
}

Eigen::VectorXd AbstractDDPSolver::GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const
{
    Eigen::VectorXd u = U_ref_.col(t) + k_gains_[t] + K_gains_[t] * dynamics_solver_->StateDelta(x, X_ref_.col(t));
    return u.cwiseMax(dynamics_solver_->get_control_limits().col(0)).cwiseMin(dynamics_solver_->get_control_limits().col(1));
}

}  // namespace exotica
