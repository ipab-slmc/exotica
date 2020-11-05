//
// Copyright (c) 2019-2020, LAAS-CNRS, University of Edinburgh, University of Oxford
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

#include <exotica_ddp_solver/feasibility_driven_ddp_solver.h>

REGISTER_MOTIONSOLVER_TYPE("FeasibilityDrivenDDPSolver", exotica::FeasibilityDrivenDDPSolver)

namespace exotica
{
void FeasibilityDrivenDDPSolver::Instantiate(const FeasibilityDrivenDDPSolverInitializer& init)
{
    parameters_ = init;
    base_parameters_ = AbstractDDPSolverInitializer(FeasibilityDrivenDDPSolverInitializer(parameters_));

    clamp_to_control_limits_in_forward_pass_ = base_parameters_.ClampControlsInForwardPass;
    initial_regularization_rate_ = base_parameters_.RegularizationRate;
    th_stepinc_ = base_parameters_.ThresholdRegularizationIncrease;
    th_stepdec_ = base_parameters_.ThresholdRegularizationDecrease;

    th_stop_ = parameters_.GradientToleranceConvergenceThreshold;
    th_gradient_tolerance_ = parameters_.GradientTolerance;
    th_acceptstep_ = parameters_.DescentStepAcceptanceThreshold;
    th_acceptnegstep_ = parameters_.AscentStepAcceptanceThreshold;
}

void AbstractFeasibilityDrivenDDPSolver::AllocateData()
{
    // TODO: -1 here because of copy-paste and different conventions...
    // Crocoddyl: T running models + 1 terminal model
    // Exotica: T models ("T-1 running models")
    const int T = prob_->get_T() - 1;
    Vxx_.resize(T + 1);
    Vx_.resize(T + 1);
    Qxx_.resize(T);
    Qxu_.resize(T);
    Quu_.resize(T);
    Qx_.resize(T);
    Qu_.resize(T);
    K_.resize(T);
    k_.resize(T);
    fs_.resize(T + 1);

    xs_.resize(T + 1);
    us_.resize(T);
    xs_try_.resize(T + 1);
    us_try_.resize(T);
    dx_.resize(T + 1);

    FuTVxx_p_.resize(T);
    Quu_ldlt_.resize(T);
    Quuk_.resize(T);

    for (int t = 0; t < T; ++t)
    {
        Vxx_[t] = Eigen::MatrixXd::Zero(NDX_, NDX_);
        Vx_[t] = Eigen::VectorXd::Zero(NDX_);
        Qxx_[t] = Eigen::MatrixXd::Zero(NDX_, NDX_);
        Qxu_[t] = Eigen::MatrixXd::Zero(NDX_, NU_);
        Quu_[t] = Eigen::MatrixXd::Zero(NU_, NU_);
        Qx_[t] = Eigen::VectorXd::Zero(NDX_);
        Qu_[t] = Eigen::VectorXd::Zero(NU_);
        K_[t] = Eigen::MatrixXd::Zero(NU_, NDX_);
        k_[t] = Eigen::VectorXd::Zero(NU_);
        fs_[t] = Eigen::VectorXd::Zero(NDX_);

        if (t == 0)
        {
            xs_try_[t] = prob_->get_X(0);
        }
        else
        {
            xs_try_[t].setZero(NX_);
        }
        us_try_[t].setZero(NU_);
        dx_[t] = Eigen::VectorXd::Zero(NDX_);

        FuTVxx_p_[t] = Eigen::MatrixXd::Zero(NU_, NDX_);
        Quu_ldlt_[t] = Eigen::LDLT<Eigen::MatrixXd>(NU_);
        Quuk_[t] = Eigen::VectorXd(NU_);
    }
    Vxx_.back() = Eigen::MatrixXd::Zero(NDX_, NDX_);
    Vx_.back() = Eigen::VectorXd::Zero(NDX_);
    xs_try_.back().setZero(NX_);
    fs_.back() = Eigen::VectorXd::Zero(NDX_);

    FxTVxx_p_ = Eigen::MatrixXd::Zero(NDX_, NDX_);
    fTVxx_p_ = Eigen::VectorXd::Zero(NDX_);

    Quu_inv_.assign(T, Eigen::MatrixXd(NU_, NU_));
    fx_.assign(T, Eigen::MatrixXd(NDX_, NDX_));
    fu_.assign(T, Eigen::MatrixXd(NDX_, NU_));

    // If T changed, we need to re-allocate.
    last_T_ = T_;
}

void AbstractFeasibilityDrivenDDPSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    AbstractDDPSolver::SpecifyProblem(pointer);

    T_ = prob_->get_T();
    dt_ = dynamics_solver_->get_dt();

    NU_ = prob_->GetScene()->get_num_controls();
    NX_ = prob_->GetScene()->get_num_state();              // State vector size
    NDX_ = prob_->GetScene()->get_num_state_derivative();  // Tangent vector size

    AllocateData();
}

void AbstractFeasibilityDrivenDDPSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    T_ = prob_->get_T();
    if (T_ != last_T_) AllocateData();

    dt_ = dynamics_solver_->get_dt();
    control_limits_ = dynamics_solver_->get_control_limits();

    const Eigen::MatrixXd& X_warm = prob_->get_X();
    const Eigen::MatrixXd& U_warm = prob_->get_U();
    for (int t = 0; t < T_ - 1; ++t)
    {
        xs_[t] = X_warm.col(t);
        us_[t] = U_warm.col(t);
    }
    xs_[0] = prob_->ApplyStartState();  // Apply start state
    xs_.back() = X_warm.col(T_ - 1);
    is_feasible_ = false;  // We assume the first iteration is always infeasible. TODO: Make this configurable

    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    control_cost_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());
    steplength_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());
    regularization_evolution_.assign(GetNumberOfMaxIterations() + 1, std::numeric_limits<double>::quiet_NaN());
    prob_->PreUpdate();
    solution.resize(T_ - 1, NU_);

    // Initial roll-out to get initial cost
    cost_ = 0.0;
    control_cost_ = 0.0;
    for (int t = 0; t < T_ - 1; ++t)
    {
        prob_->Update(xs_[t], us_[t], t);
        control_cost_ += dt_ * prob_->GetControlCost(t);
        cost_ += dt_ * prob_->GetStateCost(t);
    }
    // Reset shooting nodes so we can warm-start from state trajectory
    prob_->set_X(X_warm);
    cost_ += prob_->GetStateCost(T_ - 1) + control_cost_;
    prob_->SetCostEvolution(0, cost_);
    control_cost_evolution_.at(0) = control_cost_;

    xreg_ = std::max(regmin_, initial_regularization_rate_);
    ureg_ = std::max(regmin_, initial_regularization_rate_);
    was_feasible_ = false;

    bool diverged = false;
    bool converged = false;

    bool recalcDiff = true;
    int iter;

    // double time_taken_setup_ = planning_timer.GetDuration();

    for (iter = 1; iter <= GetNumberOfMaxIterations(); ++iter)
    {
        // Check whether user interrupted (Ctrl+C)
        if (Server::IsRos() && !ros::ok())
        {
            if (debug_) HIGHLIGHT("Solving cancelled by user");
            prob_->termination_criterion = TerminationCriterion::UserDefined;
            break;
        }

        backward_pass_timer.Reset();
        while (!ComputeDirection(recalcDiff))
        {
            // HIGHLIGHT("Increase regularization in ComputeDirection and try again.")
            recalcDiff = false;
            IncreaseRegularization();
            if (xreg_ == regmax_)
            {
                if (debug_) WARNING_NAMED("FeasibilityDrivenDDPSolver::Solve", "State regularization exceeds maximum regularization: " << xreg_ << " > " << regmax_)
                diverged = true;
                break;
            }
            else
            {
                continue;
            }
        }
        time_taken_backward_pass_ = backward_pass_timer.GetDuration();

        if (diverged)
        {
            if (debug_) WARNING("Terminating: Divergence in ComputeDirection / BackwardPass.");
            break;
        }

        UpdateExpectedImprovement();

        // We need to recalculate the derivatives when the step length passes
        recalcDiff = false;
        line_search_timer.Reset();
        for (int ai = 0; ai < alpha_space_.size(); ++ai)
        {
            steplength_ = alpha_space_(ai);
            dV_ = TryStep(steplength_);

            ExpectedImprovement();
            dVexp_ = steplength_ * (d_[0] + 0.5 * steplength_ * d_[1]);

            if (dVexp_ >= 0)
            {  // descent direction
                if (d_[0] < th_grad_ || dV_ > th_acceptstep_ * dVexp_)
                {
                    was_feasible_ = is_feasible_;
                    SetCandidate(xs_try_, us_try_, (was_feasible_) || (steplength_ == 1.0));
                    cost_ = cost_try_;
                    control_cost_ = control_cost_try_;
                    prob_->SetCostEvolution(iter, cost_);
                    control_cost_evolution_.at(iter) = control_cost_;
                    recalcDiff = true;
                    break;
                }
            }
            else
            {  // reducing the gaps by allowing a small increment in the cost value
                if (dV_ > th_acceptnegstep_ * dVexp_)
                {
                    // if (debug_) INFO_NAMED("FDDP", "Ascent direction: " << dV_ << " > " << th_acceptnegstep_ * dVexp_)
                    was_feasible_ = is_feasible_;
                    SetCandidate(xs_try_, us_try_, (was_feasible_) || (steplength_ == 1));
                    cost_ = cost_try_;
                    control_cost_ = control_cost_try_;
                    prob_->SetCostEvolution(iter, cost_);
                    control_cost_evolution_.at(iter) = control_cost_;
                    break;
                }
                // else
                // {
                //     if (debug_) INFO_NAMED("FDDP", "Ascent direction, but not accepted: " << dV_ << " < " << th_acceptnegstep_ * dVexp_)
                // }
            }

            prob_->SetCostEvolution(iter, cost_);
            control_cost_evolution_.at(iter) = control_cost_;
        }
        time_taken_forward_pass_ = line_search_timer.GetDuration();

        steplength_evolution_.at(iter) = steplength_;
        regularization_evolution_.at(iter) = xreg_;

        if (debug_)
        {
            if (iter % 10 == 0 || iter == 1)
            {
                std::cout << "iter \t cost \t      stop \t    grad \t  xreg";
                std::cout << " \t      ureg \t step \t feas \tdV-exp \t      dV\n";
            }

            std::cout << std::setw(4) << iter << "  ";
            std::cout << std::scientific << std::setprecision(5) << cost_ << "  ";
            std::cout << stop_ << "  " << -d_[1] << "  ";
            std::cout << xreg_ << "  " << ureg_ << "   ";
            std::cout << std::fixed << std::setprecision(4) << steplength_ << "     ";
            std::cout << is_feasible_ << "  ";
            std::cout << std::scientific << std::setprecision(5) << dVexp_ << "  ";
            std::cout << dV_ << '\n';
        }

        // Adapt regularization based on step-length
        if (steplength_ > th_stepdec_)
        {
            DecreaseRegularization();
        }
        if (steplength_ <= th_stepinc_)
        {
            IncreaseRegularization();
            if (xreg_ == regmax_)
            {
                WARNING_NAMED("FeasibilityDrivenDDPSolver::Solve", "State regularization exceeds maximum regularization: " << xreg_ << " > " << regmax_)
                diverged = true;
                break;
            }
        }
        CheckStoppingCriteria();

        // Stop is only exactly zero if all dimensions of Qu_[t] have been
        // artificially set to 0. This is e.g. the case for a Control-Limited
        // variant (BoxFDDP). However, we do not want to stop at the first
        // saturation of all dimensions.
        if (was_feasible_ && stop_ < th_stop_ && stop_ != 0.0)
        {
            if (debug_) HIGHLIGHT_NAMED("FeasibilityDrivenDDPSolver::Solve", "Convergence: " << stop_ << " < " << th_stop_)
            converged = true;
            break;
        }

        if (diverged)
        {
            WARNING_NAMED("FeasibilityDrivenDDPSolver::Solve", "Terminating: Divergence in ForwardPass.");
            break;
        }

        // Check gradient tolerance
        if (was_feasible_ && std::abs(-d_[1]) < th_gradient_tolerance_)
        {
            if (debug_) HIGHLIGHT_NAMED("FeasibilityDrivenDDPSolver::Solve", "Gradient tolerance: " << -d_[1] << " < " << th_gradient_tolerance_)
            prob_->termination_criterion = TerminationCriterion::GradientTolerance;
            break;
        }

        prob_->OnSolverIterationEnd();
    }

    if (diverged) prob_->termination_criterion = TerminationCriterion::Divergence;
    if (converged) prob_->termination_criterion = TerminationCriterion::Convergence;
    if (!converged && iter == GetNumberOfMaxIterations() + 1) prob_->termination_criterion = TerminationCriterion::IterationLimit;

    // Store the best solution found over all iterations
    for (int t = 0; t < T_ - 1; ++t)
    {
        solution.row(t) = us_[t].transpose();
        // prob_->Update(us_[t], t);
    }

    planning_time_ = planning_timer.GetDuration();

    // HIGHLIGHT(std::setprecision(4) << "Setup: " << time_taken_setup_ * 1e3 << "\tBwd: " << time_taken_backward_pass_ * 1e3 << "\tFwd: " << time_taken_forward_pass_ * 1e3 << "\tSolve = " << (planning_time_ - time_taken_setup_)*1e3 << "\talpha=" << steplength_)
}

void AbstractFeasibilityDrivenDDPSolver::IncreaseRegularization()
{
    xreg_ *= regfactor_;
    if (xreg_ > regmax_)
    {
        xreg_ = regmax_;
    }
    ureg_ = xreg_;
}

void AbstractFeasibilityDrivenDDPSolver::DecreaseRegularization()
{
    xreg_ /= regfactor_;
    if (xreg_ < regmin_)
    {
        xreg_ = regmin_;
    }
    ureg_ = xreg_;
}

double AbstractFeasibilityDrivenDDPSolver::CheckStoppingCriteria()
{
    stop_ = 0.;
    for (int t = 0; t < T_ - 1; ++t)
    {
        stop_ += Qu_[t].squaredNorm();
    }
    return stop_;
}

const Eigen::Vector2d& AbstractFeasibilityDrivenDDPSolver::ExpectedImprovement()
{
    dv_ = 0;
    if (!is_feasible_)
    {
        for (int t = 0; t < T_; ++t)
        {
            dx_[t] = prob_->GetScene()->GetDynamicsSolver()->StateDelta(xs_[t], xs_try_[t]);
            fTVxx_p_.noalias() = Vxx_[t] * dx_[t];
            dv_ -= fs_[t].dot(fTVxx_p_);
        }
    }
    d_[0] = dg_ + dv_;
    d_[1] = dq_ - 2 * dv_;
    return d_;
}

void AbstractFeasibilityDrivenDDPSolver::UpdateExpectedImprovement()
{
    dg_ = 0;
    dq_ = 0;
    if (!is_feasible_)
    {
        dg_ -= Vx_.back().dot(fs_.back());
        fTVxx_p_.noalias() = Vxx_.back() * fs_.back();
        dq_ += fs_.back().dot(fTVxx_p_);
    }
    for (int t = 0; t < T_ - 1; ++t)
    {
        dg_ += Qu_[t].dot(k_[t]);
        dq_ -= k_[t].dot(Quuk_[t]);
        if (!is_feasible_)
        {
            dg_ -= Vx_[t].dot(fs_[t]);
            fTVxx_p_.noalias() = Vxx_[t] * fs_[t];
            dq_ += fs_[t].dot(fTVxx_p_);
        }
    }
}

void AbstractFeasibilityDrivenDDPSolver::SetCandidate(const std::vector<Eigen::VectorXd>& xs_in, const std::vector<Eigen::VectorXd>& us_in, const bool is_feasible)
{
    const std::size_t T = static_cast<std::size_t>(prob_->get_T());

    if (xs_in.size() == 0)
    {
        for (int t = 0; t < T_; ++t)
        {
            xs_[t].setZero(NX_);
        }
    }
    else
    {
        if (xs_in.size() != T)
        {
            ThrowPretty("Warm start state has wrong dimension, got " << xs_in.size() << " expecting " << (T + 1));
        }
        std::copy(xs_in.begin(), xs_in.end(), xs_.begin());
    }

    if (us_in.size() == 0)
    {
        for (int t = 0; t < T_ - 1; ++t)
        {
            us_[t].setZero(NU_);
        }
    }
    else
    {
        if (us_in.size() != T - 1)
        {
            ThrowPretty("Warm start control has wrong dimension, got " << us_in.size() << " expecting " << T);
        }
        std::copy(us_in.begin(), us_in.end(), us_.begin());
    }
    is_feasible_ = is_feasible;
}

double AbstractFeasibilityDrivenDDPSolver::TryStep(const double steplength)
{
    ForwardPass(steplength);
    return cost_ - cost_try_;
}

void AbstractFeasibilityDrivenDDPSolver::ForwardPass(const double steplength)
{
    if (steplength > 1. || steplength < 0.)
    {
        ThrowPretty("Invalid argument: invalid step length, value should be between 0. to 1. - got=" << steplength);
    }
    cost_try_ = 0.;
    control_cost_try_ = 0.;
    xnext_ = prob_->get_X(0);
    for (int t = 0; t < T_ - 1; ++t)
    {
        // If feasible or the gaps are closed, start the shooting from the previous step.
        if ((is_feasible_) || (steplength == 1))
        {
            xs_try_[t] = xnext_;
        }
        else
        {
            // We need to obtain a state based on the expected reduction of the gap given the step length (dt=unit time)
            prob_->GetScene()->GetDynamicsSolver()->Integrate(xnext_, fs_[t] * (steplength - 1), 1., xs_try_[t]);
        }
        dx_[t] = prob_->GetScene()->GetDynamicsSolver()->StateDelta(xs_try_[t], xs_[t]);  // NB: StateDelta in Exotica is the other way round than in Pinocchio
        us_try_[t].noalias() = us_[t] - k_[t] * steplength - K_[t] * dx_[t];              // This is weird. It works better WITHOUT the feedback ?!

        if (clamp_to_control_limits_in_forward_pass_)
        {
            us_try_[t] = us_try_[t].cwiseMax(control_limits_.col(0)).cwiseMin(control_limits_.col(1));
        }

        prob_->Update(xs_try_[t], us_try_[t], t);  // Performs integration and update of cost
        xnext_ = prob_->get_X(t + 1);

        control_cost_try_ += dt_ * prob_->GetControlCost(t);
        cost_try_ += dt_ * prob_->GetStateCost(t);

        if (IsNaN(cost_try_))
        {
            WARNING_NAMED("NaN in ForwardPass", "forward_error - NaN in cost_try_ at t=" << t);
            return;
        }
        if (IsNaN(xnext_.lpNorm<Eigen::Infinity>()))
        {
            WARNING_NAMED("NaN in ForwardPass", "forward_error - xnext_ isn't finite at t=" << t << ": x=" << xs_try_[t].transpose() << ", u=" << us_try_[t].transpose() << ", xnext=" << xnext_.transpose());
            return;
        }
    }

    // Terminal model
    if ((is_feasible_) || (steplength == 1))
    {
        xs_try_.back() = xnext_;
    }
    else
    {
        prob_->GetScene()->GetDynamicsSolver()->Integrate(xnext_, fs_.back() * (steplength - 1), 1., xs_try_.back());
    }
    prob_->UpdateTerminalState(xs_try_.back());
    cost_try_ += prob_->GetStateCost(T_ - 1) + control_cost_try_;

    if (IsNaN(cost_try_))
    {
        WARNING_NAMED("NaN in ForwardPass", "forward_error - cost NaN");
        return;
    }
}

bool AbstractFeasibilityDrivenDDPSolver::ComputeDirection(const bool recalcDiff)
{
    if (recalcDiff)
    {
        CalcDiff();
    }
    return BackwardPassFDDP();
}

double AbstractFeasibilityDrivenDDPSolver::CalcDiff()
{
    cost_ = 0;
    control_cost_ = 0;
    // Running cost
    for (int t = 0; t < T_ - 1; ++t)
    {
        control_cost_ += dt_ * prob_->GetControlCost(t);
        cost_ += dt_ * prob_->GetStateCost(t);
    }
    // Terminal cost
    cost_ += prob_->GetStateCost(T_ - 1) + control_cost_;

    if (!is_feasible_)
    {
        // Defects for t=0..T
        for (int t = 0; t < prob_->get_T(); ++t)
        {
            fs_[t] = prob_->GetScene()->GetDynamicsSolver()->StateDelta(prob_->get_X(t), xs_[t]);  // Exotica's convention differs...
        }
    }
    else if (!was_feasible_)
    {  // closing the gaps
        for (std::vector<Eigen::VectorXd>::iterator it = fs_.begin(); it != fs_.end(); ++it)
        {
            it->setZero();
        }
    }
    return cost_;
}

bool AbstractFeasibilityDrivenDDPSolver::BackwardPassFDDP()
{
    Vxx_.back() = prob_->GetStateCostHessian(T_ - 1);
    Vx_.back() = prob_->GetStateCostJacobian(T_ - 1);

    if (!std::isnan(xreg_))
    {
        Vxx_.back().diagonal().array() += xreg_;
    }

    if (!is_feasible_)
    {
        Vx_.back().noalias() += Vxx_.back() * fs_.back();
    }

    for (int t = static_cast<int>(prob_->get_T()) - 2; t >= 0; --t)
    {
        const Eigen::MatrixXd& Vxx_p = Vxx_[t + 1];
        const Eigen::VectorXd& Vx_p = Vx_[t + 1];

        Qxx_[t].noalias() = dt_ * prob_->GetStateCostHessian(t);
        Qxu_[t].noalias() = dt_ * prob_->GetStateControlCostHessian().transpose();
        Quu_[t].noalias() = dt_ * prob_->GetControlCostHessian(t);
        Qx_[t].noalias() = dt_ * prob_->GetStateCostJacobian(t);
        Qu_[t].noalias() = dt_ * prob_->GetControlCostJacobian(t);

        dynamics_solver_->ComputeDerivatives(xs_[t], us_[t]);
        fx_[t].noalias() = dynamics_solver_->get_Fx();
        fu_[t].noalias() = dynamics_solver_->get_Fu();

        FxTVxx_p_.noalias() = fx_[t].transpose() * Vxx_p;
        FuTVxx_p_[t].noalias() = fu_[t].transpose() * Vxx_p;
        Qxx_[t].noalias() += FxTVxx_p_ * fx_[t];
        Qxu_[t].noalias() += FxTVxx_p_ * fu_[t];
        Quu_[t].noalias() += FuTVxx_p_[t] * fu_[t];
        Qx_[t].noalias() += fx_[t].transpose() * Vx_p;
        Qu_[t].noalias() += fu_[t].transpose() * Vx_p;

        if (!std::isnan(ureg_))
        {
            Quu_[t].diagonal().array() += ureg_;
        }

        ComputeGains(t);

        Vx_[t] = Qx_[t];
        if (std::isnan(ureg_))
        {
            Vx_[t].noalias() -= K_[t].transpose() * Qu_[t];
        }
        else
        {
            Quuk_[t].noalias() = Quu_[t] * k_[t];
            Vx_[t].noalias() += K_[t].transpose() * Quuk_[t];
            Vx_[t].noalias() -= 2 * (K_[t].transpose() * Qu_[t]);
        }
        Vxx_[t] = Qxx_[t];
        Vxx_[t].noalias() -= Qxu_[t] * K_[t];
        Vxx_[t] = 0.5 * (Vxx_[t] + Vxx_[t].transpose()).eval();

        if (!std::isnan(xreg_))
        {
            Vxx_[t].diagonal().array() += xreg_;
        }

        // Compute and store the Vx gradient at end of the interval (rollout state)
        if (!is_feasible_)
        {
            Vx_[t].noalias() += Vxx_[t] * fs_[t];
        }

        if (IsNaN(Vx_[t].lpNorm<Eigen::Infinity>()))
        {
            HIGHLIGHT("Vx_[" << t << "] is NaN: " << Vx_[t].transpose());
            return false;
        }
        if (IsNaN(Vxx_[t].lpNorm<Eigen::Infinity>()))
        {
            HIGHLIGHT("Vxx_[" << t << "] is NaN");
            return false;
        }
    }
    return true;
}

void AbstractFeasibilityDrivenDDPSolver::ComputeGains(const int t)
{
    // Quu_inv_[t].noalias() = Quu_[t].inverse();
    // K_[t].noalias() = Quu_inv_[t] * Qxu_[t].transpose();
    // k_[t].noalias() = Quu_inv_[t] * Qu_[t];
    // return;

    while (true)
    {
        Quu_ldlt_[t].compute(Quu_[t]);
        const Eigen::ComputationInfo& info = Quu_ldlt_[t].info();
        if (info != Eigen::Success)
        {
            HIGHLIGHT_NAMED("ComputeGains", "Cholesky failed for reg=" << ureg_ << ", Quu_[t]=\n"
                                                                       << Quu_[t])
            Quu_[t].diagonal().array() -= ureg_;
            IncreaseRegularization();
            Quu_[t].diagonal().array() += ureg_;
            if (ureg_ == regmax_) ThrowPretty("backward_error - error in Cholesky decomposition\n"
                                              << Quu_[t]);
            // ThrowPretty("backward_error - error in Cholesky decomposition\n"
            //             << Quu_[t]);
        }
        else
        {
            break;
        }
    }
    K_[t] = Qxu_[t].transpose();
    Quu_ldlt_[t].solveInPlace(K_[t]);
    k_[t] = Qu_[t];
    Quu_ldlt_[t].solveInPlace(k_[t]);
}

}  // namespace exotica
