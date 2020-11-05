//
// Copyright (c) 2019-2020, University of Edinburgh, University of Oxford
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
    NU_ = prob_->GetScene()->get_num_controls();
    NX_ = prob_->GetScene()->get_num_state();
    NDX_ = prob_->GetScene()->get_num_state_derivative();
    NV_ = prob_->GetScene()->get_num_velocities();
    dt_ = dynamics_solver_->get_dt();
    lambda_ = base_parameters_.RegularizationRate;
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->PreUpdate();
    solution.resize(T_ - 1, NU_);

    // Resizing and allocating logged variables
    control_cost_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());
    steplength_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());
    regularization_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());

    // Perform initial roll-out
    cost_ = 0.0;
    control_cost_ = 0.0;
    for (int t = 0; t < T_ - 1; ++t)
    {
        prob_->Update(prob_->get_U(t), t);

        // Running cost
        control_cost_ += dt_ * prob_->GetControlCost(t);
        cost_ += dt_ * prob_->GetStateCost(t);
    }

    // Add terminal cost
    cost_ += prob_->GetStateCost(T_ - 1) + control_cost_;
    prob_->SetCostEvolution(0, cost_);
    control_cost_evolution_.at(0) = control_cost_;

    // Initialize gain matrices
    K_.assign(T_, Eigen::MatrixXd(NU_, NDX_));
    k_.assign(T_, Eigen::VectorXd(NU_));

    // Allocate memory by resizing commonly reused matrices:
    X_ref_.assign(T_, Eigen::VectorXd::Zero(NX_));
    U_ref_.assign(T_ - 1, Eigen::VectorXd::Zero(NU_));
    X_try_.assign(T_, Eigen::VectorXd::Zero(NX_));
    U_try_.assign(T_ - 1, Eigen::VectorXd::Zero(NU_));
    for (int t = 0; t < T_; ++t)
    {
        X_ref_[t] = prob_->get_X(t);
        X_try_[t] = prob_->get_X(t);
    }
    for (int t = 0; t < T_ - 1; ++t)
    {
        U_ref_[t] = prob_->get_U(t);
        U_try_[t] = prob_->get_U(t);
    }

    Vxx_.assign(T_, Eigen::MatrixXd::Zero(NDX_, NDX_));
    Vx_.assign(T_, Eigen::VectorXd::Zero(NDX_));

    Qx_.assign(T_ - 1, Eigen::VectorXd::Zero(NDX_));
    Qu_.assign(T_ - 1, Eigen::VectorXd::Zero(NU_));
    Qxx_.assign(T_ - 1, Eigen::MatrixXd::Zero(NDX_, NDX_));
    Qux_.assign(T_ - 1, Eigen::MatrixXd::Zero(NU_, NDX_));
    Quu_.assign(T_ - 1, Eigen::MatrixXd::Zero(NU_, NU_));
    Quu_inv_.assign(T_ - 1, Eigen::MatrixXd::Zero(NU_, NU_));
    fx_.assign(T_ - 1, Eigen::MatrixXd::Zero(NDX_, NDX_));
    fu_.assign(T_ - 1, Eigen::MatrixXd::Zero(NDX_, NU_));

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
            rollout_cost = ForwardPass(alpha);

            // TODO: More advanced line-search acceptance
            if (rollout_cost < cost_)
            {
                cost_ = rollout_cost;
                control_cost_ = control_cost_try_;
                for (int t = 0; t < T_ - 1; ++t) U_try_[t] = prob_->get_U(t);
                alpha_best_ = alpha;
                break;
            }
        }
        time_taken_forward_pass_ = line_search_timer.GetDuration();

        // Finiteness checks
        // if (!U_try_.allFinite())
        // {
        //     prob_->termination_criterion = TerminationCriterion::Divergence;
        //     WARNING_NAMED("DDPSolver", "Divergence: Controls are non-finite");
        //     return;
        // }
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

        steplength_evolution_.at(iteration) = alpha_best_;
        regularization_evolution_.at(iteration) = lambda_;

        // If better than previous iteration, copy solutions for next iteration
        if (cost_ < cost_prev_)
        {
            cost_prev_ = cost_;
            U_ref_ = U_try_;

            if (alpha_best_ < base_parameters_.ThresholdRegularizationDecrease)
            {
                IncreaseRegularization();
            }
            else if (alpha_best_ > base_parameters_.ThresholdRegularizationIncrease)
            {
                if (lambda_ > base_parameters_.MinimumRegularization) DecreaseRegularization();
            }
        }
        else
        {
            cost_ = cost_prev_;
            control_cost_ = control_cost_evolution_.at(iteration - 1);
            // Revert by not storing U_try_ as U_ref_ (maintain U_ref_)

            IncreaseRegularization();
        }

        // Roll-out and store reference state trajectory
        // TODO: This roll-out may not be required => The line-search already does a roll-out.
        for (int t = 0; t < T_ - 1; ++t)
            prob_->Update(U_ref_[t], t);
        for (int t = 0; t < T_; ++t)
            X_ref_[t] = prob_->get_X(t);

        prob_->SetCostEvolution(iteration, cost_);
        control_cost_evolution_.at(iteration) = control_cost_;

        // Iteration limit
        if (iteration == GetNumberOfMaxIterations())
        {
            if (debug_) HIGHLIGHT_NAMED("DDPSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());
            prob_->termination_criterion = TerminationCriterion::IterationLimit;
        }

        prob_->OnSolverIterationEnd();
    }

    // Store the best solution found over all iterations
    for (int t = 0; t < T_ - 1; ++t)
    {
        solution.row(t) = U_ref_[t].transpose();
        prob_->Update(U_ref_[t], t);  // TODO: This roll-out may also not be required...
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

double AbstractDDPSolver::ForwardPass(const double alpha)
{
    cost_try_ = 0.0;
    control_cost_try_ = 0.0;

    Eigen::VectorXd u_hat(NU_);  // TODO: allocate outside

    for (int t = 0; t < T_ - 1; ++t)
    {
        u_hat = U_ref_[t];
        u_hat.noalias() += alpha * k_[t];
        u_hat.noalias() += K_[t] * dynamics_solver_->StateDelta(prob_->get_X(t), X_ref_[t]);

        // Clamp controls, if desired:
        if (base_parameters_.ClampControlsInForwardPass)
        {
            u_hat = u_hat.cwiseMax(dynamics_solver_->get_control_limits().col(0)).cwiseMin(dynamics_solver_->get_control_limits().col(1));
        }

        prob_->Update(u_hat, t);
        control_cost_try_ += dt_ * prob_->GetControlCost(t);
        cost_try_ += dt_ * prob_->GetStateCost(t);
    }

    // add terminal cost
    cost_try_ += prob_->GetStateCost(T_ - 1) + control_cost_try_;
    return cost_try_;
}

Eigen::VectorXd AbstractDDPSolver::GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const
{
    Eigen::VectorXd u = U_ref_[t] + k_[t] + K_[t] * dynamics_solver_->StateDelta(x, X_ref_[t]);
    return u.cwiseMax(dynamics_solver_->get_control_limits().col(0)).cwiseMin(dynamics_solver_->get_control_limits().col(1));
}

const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_Vxx() const { return Vxx_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_Vx() const { return Vx_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_Qxx() const { return Qxx_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_Qux() const { return Qux_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_Quu() const { return Quu_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_Qx() const { return Qx_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_Qu() const { return Qu_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_K() const { return K_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_k() const { return k_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_X_try() const { return X_try_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_U_try() const { return U_try_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_X_ref() const { return X_ref_; }
const std::vector<Eigen::VectorXd>& AbstractDDPSolver::get_U_ref() const { return U_ref_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_Quu_inv() const { return Quu_inv_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_fx() const { return fx_; }
const std::vector<Eigen::MatrixXd>& AbstractDDPSolver::get_fu() const { return fu_; }
std::vector<double> AbstractDDPSolver::get_control_cost_evolution() const
{
    std::vector<double> ret;
    ret.reserve(control_cost_evolution_.size());
    for (size_t position = 0; position < control_cost_evolution_.size(); ++position)
    {
        if (std::isnan(control_cost_evolution_[position])) break;
        ret.push_back(control_cost_evolution_[position]);
    }
    return ret;
}

void AbstractDDPSolver::set_control_cost_evolution(const int index, const double cost)
{
    if (index > -1 && index < static_cast<int>(control_cost_evolution_.size()))
    {
        control_cost_evolution_[index] = cost;
    }
    else if (index == -1)
    {
        control_cost_evolution_[control_cost_evolution_.size() - 1] = cost;
    }
    else
    {
        ThrowPretty("Out of range: " << index << " where length=" << control_cost_evolution_.size());
    }
}

std::vector<double> AbstractDDPSolver::get_steplength_evolution() const
{
    std::vector<double> ret;
    ret.reserve(steplength_evolution_.size());
    for (size_t position = 1; position < steplength_evolution_.size(); ++position)
    {
        if (std::isnan(steplength_evolution_[position])) break;
        ret.push_back(steplength_evolution_[position]);
    }
    return ret;
}

std::vector<double> AbstractDDPSolver::get_regularization_evolution() const
{
    std::vector<double> ret;
    ret.reserve(regularization_evolution_.size());
    for (size_t position = 1; position < regularization_evolution_.size(); ++position)
    {
        if (std::isnan(regularization_evolution_[position])) break;
        ret.push_back(regularization_evolution_[position]);
    }
    return ret;
}
}  // namespace exotica
