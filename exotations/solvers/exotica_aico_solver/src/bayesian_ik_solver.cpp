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

#include <exotica_aico_solver/bayesian_ik_solver.h>
#include <exotica_core/server.h>

REGISTER_MOTIONSOLVER_TYPE("BayesianIKSolver", exotica::BayesianIKSolver)

namespace exotica
{
void BayesianIKSolver::Instantiate(const BayesianIKSolverInitializer& init)
{
    parameters_ = init;
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

void BayesianIKSolver::SpecifyProblem(PlanningProblemPtr problem)
{
    if (problem->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        ThrowNamed("This solver can't use problem of type '" << problem->type() << "'!");
    }
    MotionSolver::SpecifyProblem(problem);
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(problem);

    InitMessages();
}

void BayesianIKSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    prob_->termination_criterion = TerminationCriterion::NotStarted;
    planning_time_ = -1;

    Eigen::VectorXd q0 = prob_->ApplyStartState();

    Timer timer;
    if (verbose_) ROS_WARN_STREAM("BayesianIKSolver: Setting up the solver");
    update_count_ = 0;
    damping = damping_init_;
    double d;
    iteration_count_ = -1;
    InitTrajectory(q0);
    if (verbose_) ROS_WARN_STREAM("BayesianIKSolver: Solving");

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
        if (sweep_ >= max_backtrack_iterations_)
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

    solution.resize(1, prob_->N);
    solution.row(0) = q;
    planning_time_ = timer.GetDuration();
}

void BayesianIKSolver::InitMessages()
{
    s = Eigen::VectorXd::Zero(prob_->N);
    Sinv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    v = Eigen::VectorXd::Zero(prob_->N);
    Vinv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    // if (use_bwd_msg_)
    // {
    //     if (bwd_msg_v_.rows() == prob_->N && bwd_msg_Vinv_.rows() == prob_->N && bwd_msg_Vinv_.cols() == prob_->N)
    //     {
    //         v[prob_->GetT() - 1] = bwd_msg_v_;
    //         Vinv[prob_->GetT() - 1] = bwd_msg_Vinv_;
    //     }
    //     else
    //     {
    //         use_bwd_msg_ = false;
    //         WARNING("Backward message initialisation skipped, matrices have incorrect dimensions.");
    //     }
    // }
    b = Eigen::VectorXd::Zero(prob_->N);
    damping_reference_ = Eigen::VectorXd::Zero(prob_->N);
    Binv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    r = Eigen::VectorXd::Zero(prob_->N);
    R = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    rhat = 0;
    qhat = Eigen::VectorXd::Zero(prob_->N);
    q = b;
}

void BayesianIKSolver::InitTrajectory(const Eigen::VectorXd& q_init)
{
    qhat = q_init;
    q = q_init;
    damping_reference_ = q_init;
    b = q_init;
    s = q_init;
    v = q_init;
    Sinv.setZero();
    Sinv.diagonal().setConstant(damping);
    Vinv.setZero();
    Vinv.diagonal().setConstant(damping);

    // W is still writable, check dimension
    if (prob_->W.rows() != prob_->N)
    {
        ThrowNamed(prob_->W.rows() << "!=" << prob_->N);
    }

    // Set constant W,Win,H,Hinv
    W = prob_->W;
    Winv = W.inverse();

    // Compute task message reference
    UpdateTaskMessage(b, 0.0);

    cost_ = EvaluateTrajectory(b, true);  // The problem will be updated via UpdateTaskMessage, i.e. do not update on this roll-out
    cost_prev_ = cost_;
    prob_->SetCostEvolution(0, cost_);
    if (cost_ < 0) ThrowNamed("Invalid cost! " << cost_);
    if (verbose_) HIGHLIGHT("Initial cost, updates: " << update_count_ << ", cost: " << cost_);
    RememberOldState();
}

void BayesianIKSolver::UpdateFwdMessage()
{
    Eigen::MatrixXd barS(prob_->N, prob_->N), St;
    inverseSymPosDef(barS, Sinv + R);
    s = barS * (Sinv * s + r);
    St = Winv + barS;
    inverseSymPosDef(Sinv, St);
}

void BayesianIKSolver::UpdateBwdMessage()
{
    Eigen::MatrixXd barV(prob_->N, prob_->N), Vt;

    if (!use_bwd_msg_)
    {
        v = b;
        Vinv.diagonal().setConstant(1);
    }
    else
    {
        v = bwd_msg_v_;
        Vinv = bwd_msg_Vinv_;
    }
}

void BayesianIKSolver::UpdateTaskMessage(const Eigen::Ref<const Eigen::VectorXd>& qhat_t, double tolerance,
                                         double max_step_size)
{
    Eigen::VectorXd diff = qhat_t - qhat;
    if ((diff.array().abs().maxCoeff() < tolerance)) return;
    double nrm = diff.norm();
    if (max_step_size > 0. && nrm > max_step_size)
    {
        qhat += diff * (max_step_size / nrm);
    }
    else
    {
        qhat = qhat_t;
    }

    prob_->Update(qhat);
    ++update_count_;
    GetTaskCosts();
    // q_stat_.addw(c > 0 ? 1.0 / (1.0 + c) : 1.0, qhat_t);
}

void BayesianIKSolver::GetTaskCosts()
{
    Eigen::MatrixXd Jt;
    double prec;
    rhat = 0;
    R.setZero();
    r.setZero();
    for (int i = 0; i < prob_->cost.num_tasks; ++i)
    {
        prec = prob_->cost.rho(i);
        if (prec > 0)
        {
            const int& start = prob_->cost.indexing[i].start_jacobian;
            const int& len = prob_->cost.indexing[i].length_jacobian;
            Jt = prob_->cost.jacobian.middleRows(start, len).transpose();
            R += prec * Jt * prob_->cost.jacobian.middleRows(start, len);
            r += prec * Jt * (-prob_->cost.ydiff.segment(start, len) + prob_->cost.jacobian.middleRows(start, len) * qhat);
            rhat += prec * (-prob_->cost.ydiff.segment(start, len) + prob_->cost.jacobian.middleRows(start, len) * qhat).squaredNorm();
        }
    }
}

void BayesianIKSolver::UpdateTimestep(bool update_fwd, bool update_bwd,
                                      int max_relocation_iterations, double tolerance, bool force_relocation,
                                      double max_step_size)
{
    if (update_fwd) UpdateFwdMessage();
    if (update_bwd) UpdateBwdMessage();

    if (damping)
    {
        Binv = Sinv + Vinv + R + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
        AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r + damping * damping_reference_);
    }
    else
    {
        Binv = Sinv + Vinv + R;
        AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r);
    }

    for (int k = 0; k < max_relocation_iterations && !(Server::IsRos() && !ros::ok()); ++k)
    {
        if (!((!k && force_relocation) || (b - qhat).array().abs().maxCoeff() > tolerance)) break;

        UpdateTaskMessage(b, 0., max_step_size);

        //optional reUpdate fwd or bwd message (if the Dynamics might have changed...)
        if (update_fwd) UpdateFwdMessage();
        if (update_bwd) UpdateBwdMessage();

        if (damping)
        {
            Binv = Sinv + Vinv + R + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
            AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r + damping * damping_reference_);
        }
        else
        {
            Binv = Sinv + Vinv + R;
            AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r);
        }
    }
}

void BayesianIKSolver::UpdateTimestepGaussNewton(bool update_fwd,
                                                 bool update_bwd, int max_relocation_iterations, double tolerance,
                                                 double max_step_size)
{
    // TODO: implement UpdateTimestepGaussNewton
    ThrowNamed("Not implemented yet!");
}

double BayesianIKSolver::EvaluateTrajectory(const Eigen::VectorXd& x, bool skip_update)
{
    if (verbose_) ROS_WARN_STREAM("Evaluating, iteration " << iteration_count_ << ", sweep_ " << sweep_);
    q = x;

    // Perform update / roll-out
    if (!skip_update)
    {
        ++update_count_;
        prob_->Update(q);
    }

    // Task cost
    return prob_->GetScalarCost();
}

double BayesianIKSolver::Step()
{
    RememberOldState();
    switch (sweep_mode_)
    {
        //NOTE: the dependence on (Sweep?..:..) could perhaps be replaced by (DampingReference.N?..:..)
        case FORWARD:
            UpdateTimestep(true, false, 1, minimum_step_tolerance_, !iteration_count_, 1.);  //relocate once on fwd Sweep
            UpdateTimestep(false, true, 0, minimum_step_tolerance_, false, 1.);              //...not on bwd Sweep
            break;
        case SYMMETRIC:
            UpdateTimestep(true, false, 1, minimum_step_tolerance_, !iteration_count_, 1.);  //relocate once on fwd & bwd Sweep
            UpdateTimestep(false, true, (iteration_count_ ? 1 : 0), minimum_step_tolerance_, false, 1.);
            break;
        case LOCAL_GAUSS_NEWTON:
            //     UpdateTimestep(t, true, false, (iteration_count_ ? 5 : 1), minimum_step_tolerance_, !iteration_count_, 1.);  //relocate iteratively on
            //     UpdateTimestep(t, false, true, (iteration_count_ ? 5 : 0), minimum_step_tolerance_, false, 1.);  //...fwd & bwd Sweep
            break;
        case LOCAL_GAUSS_NEWTON_DAMPED:
            //     UpdateTimestepGaussNewton(t, true, false, (iteration_count_ ? 5 : 1),
            //                               minimum_step_tolerance_, 1.);  //GaussNewton in fwd & bwd Sweep
            //     UpdateTimestep(t, false, true, (iteration_count_ ? 5 : 0), minimum_step_tolerance_, false, 1.);
            break;
        default:
            ThrowNamed("non-existing Sweep mode");
    }
    b_step_ = std::max((b_old - b).array().abs().maxCoeff(), 0.0);
    damping_reference_ = b;
    // q is set inside of EvaluateTrajectory() function
    cost_ = EvaluateTrajectory(b);
    if (debug_) HIGHLIGHT("Iteration: " << iteration_count_ << ", Sweep: " << sweep_ << ", updates: " << update_count_ << ", cost: " << cost_ << " (dq=" << b_step_ << ", damping=" << damping << ")");
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

void BayesianIKSolver::RememberOldState()
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

    best_sweep_old_ = best_sweep_;
    b_step_old_ = b_step_;
}

void BayesianIKSolver::PerhapsUndoStep()
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
