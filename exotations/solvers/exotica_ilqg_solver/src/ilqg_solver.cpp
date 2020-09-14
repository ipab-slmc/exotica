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

#include <exotica_ilqg_solver/ilqg_solver.h>

REGISTER_MOTIONSOLVER_TYPE("ILQGSolver", exotica::ILQGSolver)

namespace exotica
{
void ILQGSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This ILQGSolver can't solve problem of type '" << pointer->type() << "'!");
    }

    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();
    if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "initialized");
}

void ILQGSolver::BackwardPass()
{
    constexpr double min_clamp_ = -1e10;
    constexpr double max_clamp_ = 1e10;
    const int T = prob_->get_T();
    const double dt = dynamics_solver_->get_dt();
    const int NU = prob_->GetScene()->get_num_controls();

    // Noise terms
    Eigen::VectorXd big_C_times_little_c = Eigen::VectorXd::Zero(NU, 1);
    Eigen::MatrixXd big_C_times_big_C = Eigen::MatrixXd::Zero(NU, NU);
    Eigen::MatrixXd little_c_times_little_c = Eigen::MatrixXd::Zero(1, 1);

    // Value function and derivatives at the final timestep
    double s0 = prob_->GetStateCost(T - 1);
    Eigen::MatrixXd s = prob_->GetStateCostJacobian(T - 1);
    Eigen::MatrixXd S = prob_->GetStateCostHessian(T - 1);

    for (int t = T - 2; t > 0; --t)
    {
        // eq. 3
        Eigen::VectorXd x = prob_->get_X(t), u = prob_->get_U(t);
        dynamics_solver_->ComputeDerivatives(x, u);
        Eigen::MatrixXd A = dynamics_solver_->get_Fx();
        Eigen::MatrixXd B = dynamics_solver_->get_Fu();

        double q0 = dt * (prob_->GetStateCost(t) + prob_->GetControlCost(t));
        // Aliases from the paper used. These are used with different names in e.g. DDPSolver.
        Eigen::MatrixXd q = dt * prob_->GetStateCostJacobian(t);
        Eigen::MatrixXd Q = dt * prob_->GetStateCostHessian(t);
        Eigen::MatrixXd r = dt * prob_->GetControlCostJacobian(t);
        Eigen::MatrixXd R = dt * prob_->GetControlCostHessian(t);
        Eigen::MatrixXd P = dt * prob_->GetStateControlCostHessian();

        Eigen::MatrixXd g = r + B.transpose() * s;
        Eigen::MatrixXd G = P + B.transpose() * S * A;
        Eigen::MatrixXd H = R + B.transpose() * S * B;

        if (parameters_.IncludeNoiseTerms)
        {
            Eigen::MatrixXd F = prob_->get_F(t);
            for (int i = 0; i < NU; ++i)
            {
                Eigen::MatrixXd C = std::sqrt(dt) * prob_->GetControlNoiseJacobian(i);
                Eigen::VectorXd c = std::sqrt(dt) * F.col(i);

                big_C_times_little_c = big_C_times_little_c + C.transpose() * S * c;
                big_C_times_big_C = big_C_times_big_C + C.transpose() * S * C;
                little_c_times_little_c = little_c_times_little_c + c.transpose() * S * c;
            }

            g = g + big_C_times_little_c;
            H = H + big_C_times_big_C;
        }

        // optimal U
        Eigen::EigenSolver<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> eig_solver(H);
        auto d = eig_solver.eigenvalues();
        auto V = eig_solver.eigenvectors();
        Eigen::MatrixXcd D = Eigen::MatrixXcd::Zero(d.size(), d.size());

        for (int i = 0; i < d.size(); ++i)
        {
            if (d[i].real() < 0)
                d[i] = 0;
            d[i] = 1. / (d[i] + parameters_.RegularizationRate);
            D(i, i) = d[i];
        }

        Eigen::MatrixXd H1 = (V * D * V.transpose()).real();

        l_gains_[t] = -H1 * g;
        L_gains_[t] = -H1 * G;

        // Recursive terms update
        S = Q + A.transpose() * S * A + L_gains_[t].transpose() * H * L_gains_[t] + L_gains_[t].transpose() * G + G.transpose() * L_gains_[t];
        s = q + A.transpose() * s + L_gains_[t].transpose() * H * l_gains_[t] +
            L_gains_[t].transpose() * g + G.transpose() * l_gains_[t];
        s0 = q0 + s0 + (l_gains_[t].transpose() * H * l_gains_[t] / 2.0 +
                        l_gains_[t].transpose() * g)(0);

        if (parameters_.IncludeNoiseTerms)
        {
            s0 = s0 + 0.5 * little_c_times_little_c(0);
        }

        // fix for large values
        S = S.unaryExpr([min_clamp_, max_clamp_](double x) -> double {
            return std::min(std::max(x, min_clamp_), max_clamp_);
        });
        s = s.unaryExpr([min_clamp_, max_clamp_](double x) -> double {
            return std::min(std::max(x, min_clamp_), max_clamp_);
        });
        s0 = std::min(std::max(s0, min_clamp_), max_clamp_);
    }
}

double ILQGSolver::ForwardPass(const double alpha, Eigen::MatrixXdRefConst ref_x, Eigen::MatrixXdRefConst ref_u)
{
    double cost = 0;
    const int T = prob_->get_T();
    const Eigen::MatrixXd control_limits = dynamics_solver_->get_control_limits();
    const double dt = dynamics_solver_->get_dt();

    // NOTE: Todorov uses the linearized system in forward simulation.
    //  We here use the full system dynamics. YMMV
    for (int t = 0; t < T - 1; ++t)
    {
        Eigen::VectorXd u = ref_u.col(t);
        // eq. 12
        Eigen::VectorXd delta_uk = l_gains_[t] +
                                   L_gains_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), ref_x.col(t));

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

void ILQGSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;
    // TODO: This is an interesting approach but might give us incorrect results.
    prob_->DisableStochasticUpdates();

    const int T = prob_->get_T();
    const int NU = prob_->GetScene()->get_num_controls();
    const int NX = prob_->GetScene()->get_num_state();
    const double dt = dynamics_solver_->get_dt();
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->PreUpdate();

    double initial_cost = 0;
    for (int t = 0; t < T - 1; ++t)
        initial_cost += dt * (prob_->GetControlCost(t) + prob_->GetStateCost(t));

    // add terminal cost
    initial_cost += prob_->GetStateCost(T - 1);
    prob_->SetCostEvolution(0, initial_cost);

    // initialize Gain matrices
    l_gains_.assign(T, Eigen::MatrixXd::Zero(NU, 1));
    L_gains_.assign(T, Eigen::MatrixXd::Zero(NU, NX));

    // all of the below are not pointers, since we want to copy over
    //  solutions across iterations
    Eigen::MatrixXd new_U, global_best_U = prob_->get_U();
    solution.resize(T - 1, NU);

    if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "Running ILQG solver for max " << parameters_.MaxIterations << " iterations");

    double last_cost = initial_cost, global_best_cost = initial_cost;
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

        // Backwards pass computes the gains
        backward_pass_timer.Reset();
        BackwardPass();
        if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "Backward pass complete in " << backward_pass_timer.GetDuration());
        // if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "Backward pass complete in " << backward_pass_timer.GetDuration());

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

            if (ai == 0 || (cost < current_cost && !std::isnan(cost)))
            {
                current_cost = cost;
                new_U = prob_->get_U();
                best_alpha = alpha;
            }
            // else if (cost > current_cost)
            // break;
        }

        // finite checks
        if (!new_U.allFinite() || !std::isfinite(current_cost))
        {
            prob_->termination_criterion = TerminationCriterion::Divergence;
            WARNING_NAMED("ILQGSolver", "Diverged!");
            return;
        }

        if (debug_)
        {
            HIGHLIGHT_NAMED("ILQGSolver", "Forward pass complete in " << line_search_timer.GetDuration() << " with cost: " << current_cost << " and alpha " << best_alpha);
            HIGHLIGHT_NAMED("ILQGSolver", "Final state: " << prob_->get_X(T - 1).transpose());
        }

        // copy solutions for next iteration
        if (global_best_cost > current_cost)
        {
            global_best_cost = current_cost;
            last_best_iteration = iteration;
            global_best_U = new_U;
            best_ref_x_ = ref_x;
            best_ref_u_ = ref_u;
        }

        if (iteration - last_best_iteration > parameters_.FunctionTolerancePatience)
        {
            if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "Early stopping criterion reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
            break;
        }

        if (last_cost - current_cost < parameters_.FunctionTolerance && last_cost - current_cost > 0)
        {
            if (debug_) HIGHLIGHT_NAMED("ILQGSolver", "Function tolerance reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::Divergence;
            break;
        }

        if (debug_ && iteration == parameters_.MaxIterations)
        {
            HIGHLIGHT_NAMED("ILQGSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::IterationLimit;
        }

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

    // TODO: See note at disable.
    prob_->EnableStochasticUpdates();
}

Eigen::VectorXd ILQGSolver::GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const
{
    const Eigen::MatrixXd control_limits = dynamics_solver_->get_control_limits();

    Eigen::VectorXd delta_uk = l_gains_[t] +
                               L_gains_[t] * dynamics_solver_->StateDelta(x, best_ref_x_.col(t));

    Eigen::VectorXd u = best_ref_u_.col(t) + delta_uk;
    return u.cwiseMax(control_limits.col(0)).cwiseMin(control_limits.col(1));
}

}  // namespace exotica
