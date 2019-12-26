//
// Copyright (c) 2018, University of Edinburgh
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

// This code is based on algorithm developed by Marc Toussaint
// M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
// http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
// Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html

/// \file aico_solver.h
/// \brief Approximate Inference Control

#include <exotica_aico_solver/aico_solver.h>
#include <exotica_core/server.h>

REGISTER_MOTIONSOLVER_TYPE("AICOSolver", exotica::AICOSolver)

namespace exotica
{
void AICOSolver::Instantiate(const AICOSolverInitializer& init)
{
    std::string mode = init.SweepMode;
    if (mode == "Forwardly")
        sweep_mode_ = FORWARD;
    else if (mode == "Symmetric")
        sweep_mode_ = SYMMETRIC;
    else if (mode == "LocalGaussNewton")
        sweep_mode_ = LOCAL_GAUSS_NEWTON;
    else if (mode == "LocalGaussNewtonDamped")
        sweep_mode_ = LOCAL_GAUSS_NEWTON_DAMPED;
    else
    {
        ThrowNamed("Unknown sweep mode '" << init.SweepMode << "'");
    }
    max_backtrack_iterations_ = init.MaxBacktrackIterations;
    minimum_step_tolerance_ = init.MinStep;
    step_tolerance_ = init.StepTolerance;
    function_tolerance_ = init.FunctionTolerance;
    damping_init_ = init.Damping;
    use_bwd_msg_ = init.UseBackwardMessage;
    verbose_ = init.Verbose;
}

AICOSolver::AICOSolver() = default;

AICOSolver::~AICOSolver() = default;

void AICOSolver::SpecifyProblem(PlanningProblemPtr problem)
{
    if (problem->type() != "exotica::UnconstrainedTimeIndexedProblem")
    {
        ThrowNamed("This solver can't use problem of type '" << problem->type() << "'!");
    }
    MotionSolver::SpecifyProblem(problem);
    prob_ = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(problem);

    InitMessages();
}

void AICOSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->PreUpdate();
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->termination_criterion = TerminationCriterion::NotStarted;
    planning_time_ = -1;

    Eigen::VectorXd q0 = prob_->ApplyStartState();
    std::vector<Eigen::VectorXd> q_init = prob_->GetInitialTrajectory();

    // If the initial value of the initial trajectory does not equal the start
    // state, assume that no initial guess is provided and fill the trajectory
    // with the start state
    if (!q0.isApprox(q_init[0]))
    {
        if (verbose_) HIGHLIGHT("AICO::Solve cold-started");
        q_init.assign(prob_->GetT(), q0);
    }
    else
    {
        if (verbose_) HIGHLIGHT("AICO::Solve called with initial trajectory guess");
    }

    prob_->SetStartState(q_init[0]);
    prob_->ApplyStartState();

    // Check if the trajectory length has changed, if so update the messages.
    if (prob_->GetT() != last_T_) InitMessages();

    Timer timer;
    if (verbose_) ROS_WARN_STREAM("AICO: Setting up the solver");
    update_count_ = 0;
    damping = damping_init_;
    double d;
    if (prob_->GetT() <= 0)
    {
        ThrowNamed("Problem has not been initialized properly: T=0!");
    }
    iteration_count_ = -1;
    InitTrajectory(q_init);
    if (verbose_) ROS_WARN_STREAM("AICO: Solving");

    // Reset sweep and iteration count
    sweep_ = 0;
    iteration_count_ = 0;
    while (iteration_count_ < GetNumberOfMaxIterations())
    {
        // Check whether user interrupted (Ctrl+C)
        if (Server::IsRos() && !ros::ok())
        {
            if (debug_) HIGHLIGHT("Solving cancelled by user");
            prob_->termination_criterion = TerminationCriterion::UserDefined;
            break;
        }

        d = Step();
        if (d < 0)
        {
            ThrowNamed("Negative step size!");
        }

        // 0. Check maximum backtrack iterations
        if (damping && sweep_ >= max_backtrack_iterations_)
        {
            if (debug_) HIGHLIGHT("Maximum backtrack iterations reached, exiting.");
            prob_->termination_criterion = TerminationCriterion::BacktrackIterationLimit;
            break;
        }

        // Check stopping criteria
        if (iteration_count_ > 1)
        {
            // Check convergence if
            //    a) damping is on and the iteration has concluded (the sweep improved the cost)
            //    b) damping is off [each sweep equals one iteration]
            if ((damping && sweep_improved_cost_) || !damping)
            {
                // 1. Check step tolerance
                // || x_t-x_t-1 || <= stepTolerance * max(1, || x_t ||)
                // TODO(#257): TODO(#256): move to Eigen::MatrixXd to make this easier to compute, in the meantime use old check
                //
                // TODO(#256): OLD TOLERANCE CHECK - TODO REMOVE
                if (d < minimum_step_tolerance_)
                {
                    if (debug_) HIGHLIGHT("Satisfied tolerance\titer=" << iteration_count_ << "\td=" << d << "\tminimum_step_tolerance=" << minimum_step_tolerance_);
                    prob_->termination_criterion = TerminationCriterion::StepTolerance;
                    break;
                }

                // 2. Check function tolerance
                // (f_t-1 - f_t) <= functionTolerance * max(1, abs(f_t))
                if ((cost_prev_ - cost_) <= function_tolerance_ * std::max(1.0, std::abs(cost_)))
                {
                    if (debug_) HIGHLIGHT("Function tolerance achieved: " << (cost_prev_ - cost_) << " <= " << function_tolerance_ * std::max(1.0, std::abs(cost_)));
                    prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
                    break;
                }
                cost_prev_ = cost_;
            }
        }
    }

    // Check whether maximum iteration count was reached
    if (iteration_count_ == GetNumberOfMaxIterations())
    {
        if (debug_) HIGHLIGHT("Maximum iterations reached");
        prob_->termination_criterion = TerminationCriterion::IterationLimit;
    }

    Eigen::MatrixXd sol(prob_->GetT(), prob_->N);
    for (int tt = 0; tt < prob_->GetT(); ++tt)
    {
        sol.row(tt) = q[tt];
    }
    solution = sol;
    planning_time_ = timer.GetDuration();
}

void AICOSolver::InitMessages()
{
    if (prob_ == nullptr) ThrowNamed("Problem definition is a NULL pointer!");

    if (prob_->N < 1)
    {
        ThrowNamed("State dimension is too small: n=" << prob_->N);
    }
    if (prob_->GetT() < 2)
    {
        ThrowNamed("Number of time steps is too small: T=" << prob_->GetT());
    }

    s.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    Sinv.assign(prob_->GetT(), Eigen::MatrixXd::Zero(prob_->N, prob_->N));
    Sinv[0].diagonal().setConstant(1e10);
    v.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    Vinv.assign(prob_->GetT(), Eigen::MatrixXd::Zero(prob_->N, prob_->N));
    if (use_bwd_msg_)
    {
        if (bwd_msg_v_.rows() == prob_->N && bwd_msg_Vinv_.rows() == prob_->N && bwd_msg_Vinv_.cols() == prob_->N)
        {
            v[prob_->GetT() - 1] = bwd_msg_v_;
            Vinv[prob_->GetT() - 1] = bwd_msg_Vinv_;
        }
        else
        {
            use_bwd_msg_ = false;
            WARNING("Backward message initialisation skipped, matrices have incorrect dimensions.");
        }
    }
    b.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    damping_reference_.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    Binv.assign(prob_->GetT(), Eigen::MatrixXd::Zero(prob_->N, prob_->N));
    Binv[0].setIdentity();
    Binv[0] = Binv[0] * 1e10;
    r.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    R.assign(prob_->GetT(), Eigen::MatrixXd::Zero(prob_->N, prob_->N));
    rhat = Eigen::VectorXd::Zero(prob_->GetT());
    qhat.assign(prob_->GetT(), Eigen::VectorXd::Zero(prob_->N));
    q = b;

    cost_control_.resize(prob_->GetT());
    cost_control_.setZero();
    cost_task_.resize(prob_->GetT());
    cost_task_.setZero();

    q_stat_.resize(prob_->GetT());
    for (int t = 0; t < prob_->GetT(); ++t)
    {
        q_stat_[t].resize(prob_->N);
    }

    // Set last_T_ to the problem T
    last_T_ = prob_->GetT();
}

void AICOSolver::InitTrajectory(const std::vector<Eigen::VectorXd>& q_init)
{
    if (q_init.size() != static_cast<std::size_t>(prob_->GetT()))
    {
        ThrowNamed("Incorrect number of timesteps provided!");
    }
    qhat = q_init;
    q = q_init;
    damping_reference_ = q_init;
    b = q_init;
    s = q_init;
    v = q_init;
    for (int t = 1; t < prob_->GetT(); ++t)
    {
        Sinv.at(t).setZero();
        Sinv.at(t).diagonal().setConstant(damping);
    }
    for (int t = 0; t < prob_->GetT(); ++t)
    {
        Vinv.at(t).setZero();
        Vinv.at(t).diagonal().setConstant(damping);
    }
    for (int t = 0; t < prob_->GetT(); ++t)
    {
        // Compute task message reference
        UpdateTaskMessage(t, b[t], 0.0);
    }

    // W is still writable, check dimension
    if (prob_->W.rows() != prob_->N)
    {
        ThrowNamed(prob_->W.rows() << "!=" << prob_->N);
    }

    // Set constant W,Win,H,Hinv
    W = prob_->W;
    Winv = W.inverse();

    cost_ = EvaluateTrajectory(b, true);  // The problem will be updated via UpdateTaskMessage, i.e. do not update on this roll-out
    cost_prev_ = cost_;
    prob_->SetCostEvolution(0, cost_);
    if (cost_ < 0) ThrowNamed("Invalid cost! " << cost_);
    if (debug_) HIGHLIGHT("Initial cost, updates: " << update_count_ << ", cost_(ctrl/task/total): " << cost_control_.sum() << "/" << cost_task_.sum() << "/" << cost_);
    RememberOldState();
}

void AICOSolver::UpdateFwdMessage(int t)
{
    Eigen::MatrixXd barS(prob_->N, prob_->N), St;
    inverseSymPosDef(barS, Sinv[t - 1] + R[t - 1]);
    s[t] = barS * (Sinv[t - 1] * s[t - 1] + r[t - 1]);
    St = Winv + barS;
    inverseSymPosDef(Sinv[t], St);
}

void AICOSolver::UpdateBwdMessage(int t)
{
    Eigen::MatrixXd barV(prob_->N, prob_->N), Vt;
    if (t < prob_->GetT() - 1)
    {
        inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]);
        v[t] = barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]);
        Vt = Winv + barV;
        inverseSymPosDef(Vinv[t], Vt);
    }
    if (t == prob_->GetT() - 1)
    {
        if (!use_bwd_msg_)
        {
            v[t] = b[t];
            Vinv[t].diagonal().setConstant(1);
        }
        else
        {
            v[prob_->GetT() - 1] = bwd_msg_v_;
            Vinv[prob_->GetT() - 1] = bwd_msg_Vinv_;
        }
    }
}

void AICOSolver::UpdateTaskMessage(int t,
                                   const Eigen::Ref<const Eigen::VectorXd>& qhat_t, double tolerance,
                                   double max_step_size)
{
    Eigen::VectorXd diff = qhat_t - qhat[t];
    if ((diff.array().abs().maxCoeff() < tolerance)) return;
    double nrm = diff.norm();
    if (max_step_size > 0. && nrm > max_step_size)
    {
        qhat[t] += diff * (max_step_size / nrm);
    }
    else
    {
        qhat[t] = qhat_t;
    }

    prob_->Update(qhat[t], t);
    ++update_count_;
    double c = GetTaskCosts(t);
    q_stat_[t].addw(c > 0 ? 1.0 / (1.0 + c) : 1.0, qhat_t);
}

double AICOSolver::GetTaskCosts(int t)
{
    double C = 0;
    Eigen::MatrixXd Jt;
    double prec;
    rhat[t] = 0;
    R[t].setZero();
    r[t].setZero();
    for (int i = 0; i < prob_->cost.num_tasks; ++i)
    {
        prec = prob_->cost.rho[t](i);
        if (prec > 0)
        {
            int start = prob_->cost.indexing[i].start_jacobian;
            int len = prob_->cost.indexing[i].length_jacobian;
            Jt = prob_->cost.jacobian[t].middleRows(start, len).transpose();
            C += prec * (prob_->cost.ydiff[t].segment(start, len)).squaredNorm();
            R[t] += prec * Jt * prob_->cost.jacobian[t].middleRows(start, len);
            r[t] += prec * Jt * (-prob_->cost.ydiff[t].segment(start, len) + prob_->cost.jacobian[t].middleRows(start, len) * qhat[t]);
            rhat[t] += prec * (-prob_->cost.ydiff[t].segment(start, len) + prob_->cost.jacobian[t].middleRows(start, len) * qhat[t]).squaredNorm();
        }
    }
    return prob_->get_ct() * C;
}

void AICOSolver::UpdateTimestep(int t, bool update_fwd, bool update_bwd,
                                int max_relocation_iterations, double tolerance, bool force_relocation,
                                double max_step_size)
{
    if (update_fwd) UpdateFwdMessage(t);
    if (update_bwd) UpdateBwdMessage(t);

    if (damping)
    {
        Binv[t] = Sinv[t] + Vinv[t] + R[t] + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
        AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t] + damping * damping_reference_[t]);
    }
    else
    {
        Binv[t] = Sinv[t] + Vinv[t] + R[t];
        AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]);
    }

    for (int k = 0; k < max_relocation_iterations && !(Server::IsRos() && !ros::ok()); ++k)
    {
        if (!((!k && force_relocation) || (b[t] - qhat[t]).array().abs().maxCoeff() > tolerance)) break;

        UpdateTaskMessage(t, b.at(t), 0., max_step_size);

        //optional reUpdate fwd or bwd message (if the Dynamics might have changed...)
        if (update_fwd) UpdateFwdMessage(t);
        if (update_bwd) UpdateBwdMessage(t);

        if (damping)
        {
            Binv[t] = Sinv[t] + Vinv[t] + R[t] + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
            AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t] + damping * damping_reference_[t]);
        }
        else
        {
            Binv[t] = Sinv[t] + Vinv[t] + R[t];
            AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]);
        }
    }
}

void AICOSolver::UpdateTimestepGaussNewton(int t, bool update_fwd,
                                           bool update_bwd, int max_relocation_iterations, double tolerance,
                                           double max_step_size)
{
    // TODO: implement UpdateTimestepGaussNewton
    ThrowNamed("Not implemented yet!");
}

double AICOSolver::EvaluateTrajectory(const std::vector<Eigen::VectorXd>& x,
                                      bool skip_update)
{
    if (verbose_) ROS_WARN_STREAM("Evaluating, iteration " << iteration_count_ << ", sweep " << sweep_);
    Timer timer;

    q = x;

    // Perform update / roll-out
    if (!skip_update)
    {
        for (int t = 0; t < prob_->GetT(); ++t)
        {
            ++update_count_;
            if (!q[t].allFinite())
            {
                ThrowNamed("q[" << t << "] is not finite: " << q[t].transpose());
            }
            prob_->Update(q[t], t);
        }
    }
    if (verbose_ && !skip_update) HIGHLIGHT("Roll-out took: " << timer.GetDuration());

    for (int t = 1; t < prob_->GetT(); ++t)
    {
        if (Server::IsRos() && !ros::ok()) return -1.0;

        // Control cost
        cost_control_(t) = prob_->GetScalarTransitionCost(t);

        // Task cost
        cost_task_(t) = prob_->GetScalarTaskCost(t);
    }

    cost_ = cost_control_.sum() + cost_task_.sum();
    return cost_;
}

double AICOSolver::Step()
{
    RememberOldState();
    int t;
    switch (sweep_mode_)
    {
        //NOTE: the dependence on (Sweep?..:..) could perhaps be replaced by (DampingReference.N?..:..)
        case FORWARD:
            for (t = 1; t < prob_->GetT(); ++t)
            {
                UpdateTimestep(t, true, false, 1, minimum_step_tolerance_, !iteration_count_, 1.);  //relocate once on fwd Sweep
            }
            for (t = prob_->GetT() - 2; t > 0; t--)
            {
                UpdateTimestep(t, false, true, 0, minimum_step_tolerance_, false, 1.);  //...not on bwd Sweep
            }
            break;
        case SYMMETRIC:
            // ROS_WARN_STREAM("Updating forward, iteration_count_ "<<iteration_count_);
            for (t = 1; t < prob_->GetT(); ++t)
            {
                UpdateTimestep(t, true, false, 1, minimum_step_tolerance_, !iteration_count_, 1.);  //relocate once on fwd & bwd Sweep
            }
            // ROS_WARN_STREAM("Updating backward, iteration_count_ "<<iteration_count_);
            for (t = prob_->GetT() - 2; t > 0; t--)
            {
                UpdateTimestep(t, false, true, (iteration_count_ ? 1 : 0), minimum_step_tolerance_, false, 1.);
            }
            break;
        case LOCAL_GAUSS_NEWTON:
            for (t = 1; t < prob_->GetT(); ++t)
            {
                UpdateTimestep(t, true, false, (iteration_count_ ? 5 : 1), minimum_step_tolerance_, !iteration_count_, 1.);  //relocate iteratively on
            }
            for (t = prob_->GetT() - 2; t > 0; t--)
            {
                UpdateTimestep(t, false, true, (iteration_count_ ? 5 : 0), minimum_step_tolerance_, false, 1.);  //...fwd & bwd Sweep
            }
            break;
        case LOCAL_GAUSS_NEWTON_DAMPED:
            for (t = 1; t < prob_->GetT(); ++t)
            {
                UpdateTimestepGaussNewton(t, true, false, (iteration_count_ ? 5 : 1),
                                          minimum_step_tolerance_, 1.);  //GaussNewton in fwd & bwd Sweep
            }
            for (t = prob_->GetT() - 2; t > 0; t--)
            {
                UpdateTimestep(t, false, true, (iteration_count_ ? 5 : 0), minimum_step_tolerance_, false, 1.);
            }
            break;
        default:
            ThrowNamed("non-existing Sweep mode");
    }
    b_step_ = 0.0;
    for (t = 0; t < static_cast<int>(b.size()); ++t)
    {
        b_step_ = std::max((b_old[t] - b[t]).array().abs().maxCoeff(), b_step_);
    }
    damping_reference_ = b;
    // q is set inside of EvaluateTrajectory() function
    cost_ = EvaluateTrajectory(b);
    if (verbose_)
    {
        HIGHLIGHT("Iteration: " << iteration_count_ << ", Sweep: " << sweep_ << ", updates: " << update_count_ << ", cost(ctrl/task/total): " << cost_control_.sum() << "/" << cost_task_.sum() << "/" << cost_ << " (dq=" << b_step_ << ", damping=" << damping << ")");
    }
    else if (debug_ && sweep_ == 0)
    {
        HIGHLIGHT("Iteration: " << iteration_count_ << ", updates: " << update_count_ << ", cost(ctrl/task/total): " << cost_control_.sum() << "/" << cost_task_.sum() << "/" << cost_ << " (dq=" << b_step_ << ", damping=" << damping << ")");
    }
    if (cost_ < 0) return -1.0;
    best_sweep_ = sweep_;

    // If damping (similar to line-search) is being used, consider reverting this step
    if (damping) PerhapsUndoStep();

    ++sweep_;
    if (sweep_improved_cost_)
    {
        // HIGHLIGHT("Sweep improved cost, increasing iteration count and resetting sweep count");
        ++iteration_count_;
        sweep_ = 0;
        prob_->SetCostEvolution(iteration_count_, cost_);
    }

    return b_step_;
}

void AICOSolver::RememberOldState()
{
    s_old = s;
    Sinv_old = Sinv;
    v_old = v;
    Vinv_old = Vinv;
    r_old = r;
    R_old = R;
    Binv_old = Binv;
    rhat_old = rhat;
    b_old = b;
    r_old = r;
    q_old = q;
    qhat_old = qhat;
    cost_old_ = cost_;
    cost_control_old_ = cost_control_;
    cost_task_old_ = cost_task_;
    best_sweep_old_ = best_sweep_;
    b_step_old_ = b_step_;
}

void AICOSolver::PerhapsUndoStep()
{
    if (cost_ > cost_old_)
    {
        sweep_improved_cost_ = false;
        damping *= 10.;
        s = s_old;
        Sinv = Sinv_old;
        v = v_old;
        Vinv = Vinv_old;
        r = r_old;
        R = R_old;
        Binv = Binv_old;
        rhat = rhat_old;
        b = b_old;
        r = r_old;
        q = q_old;
        qhat = qhat_old;
        cost_ = cost_old_;
        damping_reference_ = b_old;
        cost_control_ = cost_control_old_;
        cost_task_ = cost_task_old_;
        best_sweep_ = best_sweep_old_;
        b_step_ = b_step_old_;
        if (verbose_) HIGHLIGHT("Reverting to previous line-search step (" << best_sweep_ << ")");
    }
    else
    {
        sweep_improved_cost_ = true;
        damping /= 5.;
    }
}
}  // namespace exotica
