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

#include <exotica_ilqr_solver/ilqr_solver.h>

REGISTER_MOTIONSOLVER_TYPE("ILQRSolver", exotica::ILQRSolver)

namespace exotica
{
void ILQRSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This ILQRSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();
}

void ILQRSolver::BackwardPass()
{
    constexpr double min_clamp_ = -1e10;
    constexpr double max_clamp_ = 1e10;
    const int T = prob_->get_T();
    const double dt = dynamics_solver_->get_dt();

    const Eigen::MatrixXd& Qf = prob_->get_Qf();
    const Eigen::MatrixXd R = dt * prob_->get_R();
    const Eigen::MatrixXd& X_star = prob_->get_X_star();

    // eq. 18
    vk_gains_[T - 1] = Qf * dynamics_solver_->StateDelta(prob_->get_X(T - 1), X_star.col(T - 1));
    vk_gains_[T - 1] = vk_gains_[T - 1].unaryExpr([min_clamp_, max_clamp_](double x) -> double {
        return std::min(std::max(x, min_clamp_), max_clamp_);
    });

    Eigen::MatrixXd Sk = Qf;
    Sk = Sk.unaryExpr([min_clamp_, max_clamp_](double x) -> double {
        return std::min(std::max(x, min_clamp_), max_clamp_);
    });
    vk_gains_[T - 1] = vk_gains_[T - 1].unaryExpr([min_clamp_, max_clamp_](double x) -> double {
        return std::min(std::max(x, min_clamp_), max_clamp_);
    });

    for (int t = T - 2; t >= 0; t--)
    {
        Eigen::VectorXd x = prob_->get_X(t), u = prob_->get_U(t);
        dynamics_solver_->ComputeDerivatives(x, u);
        Eigen::MatrixXd Ak = dynamics_solver_->get_Fx(), Bk = dynamics_solver_->get_Fu(),
                        Q = dt * prob_->get_Q(t);

        // this inverse is common for all factors
        // TODO: use LLT
        const Eigen::MatrixXd _inv =
            (Eigen::MatrixXd::Identity(R.rows(), R.cols()) * parameters_.RegularizationRate + R + Bk.transpose() * Sk * Bk).inverse();

        Kv_gains_[t] = _inv * Bk.transpose();
        K_gains_[t] = _inv * Bk.transpose() * Sk * Ak;
        Ku_gains_[t] = _inv * R;
        Sk = Ak.transpose() * Sk * (Ak - Bk * K_gains_[t]) + Q;

        vk_gains_[t] = ((Ak - Bk * K_gains_[t]).transpose() * vk_gains_[t + 1]) -
                       (K_gains_[t].transpose() * R * u) + (Q * x);

        // fix for large values
        Sk = Sk.unaryExpr([min_clamp_, max_clamp_](double x) -> double {
            return std::min(std::max(x, min_clamp_), max_clamp_);
        });
        vk_gains_[t] = vk_gains_[t].unaryExpr([min_clamp_, max_clamp_](double x) -> double {
            return std::min(std::max(x, min_clamp_), max_clamp_);
        });
    }
}

double ILQRSolver::ForwardPass(const double alpha, Eigen::MatrixXdRefConst ref_x, Eigen::MatrixXdRefConst ref_u)
{
    double cost = 0;
    const int T = prob_->get_T();
    const Eigen::MatrixXd control_limits = dynamics_solver_->get_control_limits();
    const double dt = dynamics_solver_->get_dt();

    Eigen::VectorXd delta_uk(dynamics_solver_->get_num_controls()), u(dynamics_solver_->get_num_controls());
    for (int t = 0; t < T - 1; ++t)
    {
        u = ref_u.col(t);
        // eq. 12
        delta_uk = -Ku_gains_[t] * u - Kv_gains_[t] * vk_gains_[t + 1] -
                   K_gains_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), ref_x.col(t));

        u.noalias() += alpha * delta_uk;
        // clamp controls
        u = u.cwiseMax(control_limits.col(0)).cwiseMin(control_limits.col(1));

        prob_->Update(u, t);
        cost += dt * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // add terminal cost
    cost += prob_->GetStateCost(T - 1);
    return cost;
}

void ILQRSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    const int T = prob_->get_T();
    const int NU = prob_->GetScene()->get_num_controls();
    const int NX = prob_->GetScene()->get_num_state();
    const double dt = dynamics_solver_->get_dt();
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->PreUpdate();

    double cost = 0;
    for (int t = 0; t < T - 1; ++t)
    {
        prob_->Update(prob_->get_U(t), t);

        cost += dt * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // add terminal cost
    cost += prob_->GetStateCost(T - 1);
    prob_->SetCostEvolution(0, cost);

    // initialize Gain matrices
    K_gains_.assign(T, Eigen::MatrixXd(NU, NX));
    Ku_gains_.assign(T, Eigen::MatrixXd(NU, NU));
    Kv_gains_.assign(T, Eigen::MatrixXd(NU, NX));
    vk_gains_.assign(T, Eigen::MatrixXd(NX, 1));

    // all of the below are not pointers, since we want to copy over
    //  solutions across iterations
    Eigen::MatrixXd new_U = prob_->get_U();
    solution.resize(T - 1, NU);

    if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Running ILQR solver for max " << GetNumberOfMaxIterations() << " iterations");

    double cost_prev = cost;
    int last_best_iteration = 0;

    Eigen::VectorXd alpha_space = Eigen::VectorXd::LinSpaced(11, 0.0, -3.0);
    for (int ai = 0; ai < alpha_space.size(); ++ai)
    {
        alpha_space(ai) = std::pow(10.0, alpha_space(ai));
    }

    double time_taken_backward_pass = 0.0, time_taken_forward_pass = 0.0;
    for (int iteration = 1; iteration <= GetNumberOfMaxIterations(); ++iteration)
    {
        // Check whether user interrupted (Ctrl+C)
        if (Server::IsRos() && !ros::ok())
        {
            if (debug_) HIGHLIGHT("Solving cancelled by user");
            prob_->termination_criterion = TerminationCriterion::UserDefined;
            break;
        }

        // Backwards pass computes the gains
        backward_pass_timer.Reset();
        BackwardPass();
        time_taken_backward_pass = backward_pass_timer.GetDuration();

        // Forward pass to compute new control trajectory
        line_search_timer.Reset();

        // Make a copy to compare against
        Eigen::MatrixXd ref_x = prob_->get_X(),
                        ref_u = prob_->get_U();

        // Perform a backtracking line search to find the best rate
        double best_alpha = 0;
        for (int ai = 0; ai < alpha_space.size(); ++ai)
        {
            const double& alpha = alpha_space(ai);
            double rollout_cost = ForwardPass(alpha, ref_x, ref_u);

            if (rollout_cost < cost_prev && !std::isnan(rollout_cost))
            {
                cost = rollout_cost;
                new_U = prob_->get_U();
                best_alpha = alpha;
                break;
            }
        }

        // Finiteness checks
        if (!new_U.allFinite() || !std::isfinite(cost))
        {
            prob_->termination_criterion = TerminationCriterion::Divergence;
            WARNING_NAMED("ILQRSolver", "Diverged: Controls or cost are not finite.");
            return;
        }

        time_taken_forward_pass = line_search_timer.GetDuration();

        if (debug_)
        {
            HIGHLIGHT_NAMED("ILQRSolver", "Iteration " << iteration << std::setprecision(3) << ":\tBackward pass: " << time_taken_backward_pass << " s\tForward pass: " << time_taken_forward_pass << " s\tCost: " << cost << "\talpha: " << best_alpha);
        }

        //
        // Stopping criteria checks
        //

        // Relative function tolerance
        // (f_t-1 - f_t) <= functionTolerance * max(1, abs(f_t))
        if ((cost_prev - cost) < parameters_.FunctionTolerance * std::max(1.0, std::abs(cost)))
        {
            // Function tolerance patience check
            if (parameters_.FunctionTolerancePatience > 0)
            {
                if (iteration - last_best_iteration > parameters_.FunctionTolerancePatience)
                {
                    if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Early stopping criterion reached (" << cost << " < " << cost_prev << "). Time: " << planning_timer.GetDuration());
                    prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
                    break;
                }
            }
            else
            {
                if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Function tolerance reached (" << cost << " < " << cost_prev << "). Time: " << planning_timer.GetDuration());
                prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
                break;
            }
        }
        else
        {
            // Reset function tolerance patience
            last_best_iteration = iteration;
        }

        // If better than previous iteration, copy solutions for next iteration
        if (cost < cost_prev)
        {
            cost_prev = cost;
            // last_best_iteration = iteration;
            best_ref_x_ = ref_x;
            best_ref_u_ = ref_u;
        }

        // if (iteration - last_best_iteration > parameters_.FunctionTolerancePatience)
        // {
        //     if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Early stopping criterion reached. Time: " << planning_timer.GetDuration());
        //     prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
        //     break;
        // }

        // if (last_cost - current_cost < parameters_.FunctionTolerance && last_cost - current_cost > 0)
        // {
        //     if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Function tolerance reached. Time: " << planning_timer.GetDuration());
        //     prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
        //     break;
        // }

        if (debug_ && iteration == parameters_.MaxIterations)
        {
            HIGHLIGHT_NAMED("ILQRSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::IterationLimit;
        }

        // Roll-out
        for (int t = 0; t < T - 1; ++t)
            prob_->Update(new_U.col(t), t);

        // Set cost evolution
        prob_->SetCostEvolution(iteration, cost);
    }

    // store the best solution found over all iterations
    for (int t = 0; t < T - 1; ++t)
    {
        solution.row(t) = new_U.col(t).transpose();
        prob_->Update(new_U.col(t), t);
    }

    planning_time_ = planning_timer.GetDuration();
}

Eigen::VectorXd ILQRSolver::GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const
{
    const Eigen::MatrixXd control_limits = dynamics_solver_->get_control_limits();

    Eigen::VectorXd delta_uk = -Ku_gains_[t] * best_ref_u_.col(t) - Kv_gains_[t] * vk_gains_[t + 1] -
                               K_gains_[t] * dynamics_solver_->StateDelta(x, best_ref_x_.col(t));

    Eigen::VectorXd u = best_ref_u_.col(t) + delta_uk;
    return u.cwiseMax(control_limits.col(0)).cwiseMin(control_limits.col(1));
}

}  // namespace exotica
