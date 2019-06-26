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
    if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "initialized");
}

void ILQRSolver::BackwardPass()
{
    constexpr double min_clamp_ = -1e10;
    constexpr double max_clamp_ = 1e10;
    const int T = prob_->get_T();
    const double dt = dynamics_solver_->get_dt();

    const Eigen::MatrixXd Qf = prob_->get_Qf(), R = dt * prob_->get_R();
    const Eigen::MatrixXd X_star = prob_->get_X_star();

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
        Eigen::MatrixXd Ak = dynamics_solver_->fx(x, u), Bk = dynamics_solver_->fu(x, u),
                        Q = dt * prob_->get_Q(t);

        // eq. 13-16
        Ak.noalias() = Ak * dt + Eigen::MatrixXd::Identity(Ak.rows(), Ak.cols());
        Bk.noalias() = Bk * dt;
        // this inverse is common for all factors
        const Eigen::MatrixXd _inv =
            (Eigen::MatrixXd::Identity(R.rows(), R.cols()) * lambda_ + R + Bk.transpose() * Sk * Bk).inverse();

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
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    const double dt = dynamics_solver_->get_dt();

    for (int t = 0; t < T - 1; ++t)
    {
        Eigen::VectorXd u = ref_u.col(t);
        // eq. 12
        Eigen::VectorXd delta_uk = -Ku_gains_[t] * u - Kv_gains_[t] * vk_gains_[t + 1] -
                                   K_gains_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), ref_x.col(t));

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

void ILQRSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    const int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();

    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    // initialize Gain matrices
    K_gains_.assign(T, Eigen::MatrixXd(NU, NX));
    Ku_gains_.assign(T, Eigen::MatrixXd(NU, NU));
    Kv_gains_.assign(T, Eigen::MatrixXd(NU, NX));
    vk_gains_.assign(T, Eigen::MatrixXd(NX, 1));

    // all of the below are not pointers, since we want to copy over
    //  solutions across iterations
    Eigen::MatrixXd new_U, global_best_U;
    solution.resize(T, NU);

    if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Running ILQR solver for max " << parameters_.MaxIterations << " iterations");

    double last_cost = 0, global_best_cost = 0;
    int last_best_iteration = 0;

    for (int iteration = 0; iteration < GetNumberOfMaxIterations(); ++iteration)
    {
        // Backwards pass computes the gains
        backward_pass_timer.Reset();
        BackwardPass();
        // if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Backward pass complete in " << backward_pass_timer.GetDuration());
        if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Backward pass complete in " << backward_pass_timer.GetDuration() << " lambda=" << lambda_);
        
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

            if (ai == 0 || cost < current_cost)
            {
                current_cost = cost;
                new_U = prob_->get_U();
                best_alpha = alpha;
            }
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
            HIGHLIGHT_NAMED("ILQRSolver", "Lambda greater than maximum.");
            break;
        }

        if (debug_)
        {
            HIGHLIGHT_NAMED("ILQRSolver", "Forward pass complete in " << line_search_timer.GetDuration() << " with cost: " << current_cost << " and alpha " << best_alpha);
            HIGHLIGHT_NAMED("ILQRSolver", "Final state: " << prob_->get_X(T - 1).transpose());
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

        if (iteration - last_best_iteration > parameters_.FunctionTolerancePatience)
        {
            if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Early stopping criterion reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (last_cost - current_cost < parameters_.FunctionTolerance && last_cost - current_cost > 0)
        {
            if (debug_) HIGHLIGHT_NAMED("ILQRSolver", "Function tolerance reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (debug_ && iteration == parameters_.MaxIterations - 1)
            HIGHLIGHT_NAMED("ILQRSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());

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

Eigen::VectorXd ILQRSolver::GetFeedbackControl(Eigen::VectorXd x, int t) const
{
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    Eigen::VectorXd delta_uk = -Ku_gains_[t] * best_ref_u_.col(t) - Kv_gains_[t] * vk_gains_[t + 1] -
                               K_gains_[t] * dynamics_solver_->StateDelta(x, best_ref_x_.col(t));

    Eigen::VectorXd u = best_ref_u_.col(t) + delta_uk;
    return u.cwiseMax(-control_limits).cwiseMin(control_limits);
}

}  // namespace exotica
