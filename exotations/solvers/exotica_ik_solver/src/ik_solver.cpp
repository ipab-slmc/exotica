//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_ik_solver/ik_solver.h>

REGISTER_MOTIONSOLVER_TYPE("IKSolver", exotica::IKSolver)

namespace exotica
{
void IKSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        ThrowNamed("This IKSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);

    W_inv_ = prob_->W.inverse();

    // Check dimension of W_ as this is a public member of the problem, and thus, can be edited by error.
    if (W_inv_.rows() != prob_->N || W_inv_.cols() != prob_->N)
        ThrowNamed("Size of W incorrect: (" << W_inv_.rows() << ", " << W_inv_.cols() << "), when expected: (" << prob_->N << ", " << prob_->N << ")");

    // Warn if deprecated MaxStep configuration detected:
    if (parameters_.MaxStep != 0.0 && GetNumberOfMaxIterations() != 1)
        WARNING_NAMED("IKSolver", "Deprecated configuration detected: MaxStep (given: " << parameters_.MaxStep << ") only works if MaxIterations == 1 (given: " << GetNumberOfMaxIterations() << ")");

    // Set up backtracking line-search coefficients
    alpha_space_ = Eigen::VectorXd::LinSpaced(10, 1.0, 0.1);

    lambda_ = parameters_.RegularizationRate;
    th_stepinc_ = parameters_.ThresholdRegularizationIncrease;
    th_stepdec_ = parameters_.ThresholdRegularizationDecrease;
    regmax_ = parameters_.MaximumRegularization;

    th_stop_ = parameters_.GradientToleranceConvergenceThreshold;

    // Allocate variables
    q_.resize(prob_->N);
    qd_.resize(prob_->N);
    yd_.resize(prob_->cost.length_jacobian);
    cost_jacobian_.resize(prob_->cost.length_jacobian, prob_->N);
    J_pseudo_inverse_.resize(prob_->N, prob_->cost.length_jacobian);
    J_tmp_.resize(prob_->cost.length_jacobian, prob_->cost.length_jacobian);
}

void IKSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");

    Timer timer;

    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    lambda_ = parameters_.RegularizationRate;
    q_ = prob_->ApplyStartState();

    if (prob_->q_nominal.rows() == prob_->N)
    {
        WARNING("Nominal state regularization is no longer supported - please use a JointPose task-map.");
    }

    int i;
    for (i = 0; i < GetNumberOfMaxIterations(); ++i)
    {
        prob_->Update(q_);
        error_ = prob_->GetScalarCost();
        prob_->SetCostEvolution(i, error_);
        error_prev_ = error_;

        // Absolute function tolerance check
        if (error_ < parameters_.Tolerance)
        {
            prob_->termination_criterion = TerminationCriterion::FunctionTolerance;
            break;
        }

        yd_.noalias() = prob_->cost.S * prob_->cost.ydiff;
        cost_jacobian_.noalias() = prob_->cost.S * prob_->cost.jacobian;

        // Weighted Regularized Pseudo-Inverse
        //   J_pseudo_inverse_ = W_inv_ * cost_jacobian_.transpose() * ( cost_jacobian_ * W_inv_ * cost_jacobian_.transpose() + lambda_ * I ).inverse();

        bool decomposition_ok = false;
        while (!decomposition_ok)
        {
            J_tmp_.noalias() = cost_jacobian_ * W_inv_ * cost_jacobian_.transpose();
            J_tmp_.diagonal().array() += lambda_;  // Add regularisation
            J_decomposition_.compute(J_tmp_);
            if (J_decomposition_.info() != Eigen::Success)
            {
                IncreaseRegularization();
                if (lambda_ > regmax_)
                {
                    WARNING("Divergence in Cholesky decomposition :-(");
                    prob_->termination_criterion = TerminationCriterion::Divergence;
                    break;
                }
                // ThrowPretty("Error during matrix decomposition of J_tmp_ (lambda=" << lambda_ << "):\n"
                //                                                                 << J_tmp_);
            }
            else
            {
                decomposition_ok = true;
            }
        }
        J_tmp_.noalias() = J_decomposition_.solve(Eigen::MatrixXd::Identity(prob_->cost.length_jacobian, prob_->cost.length_jacobian));  // Inverse
        J_pseudo_inverse_.noalias() = W_inv_ * cost_jacobian_.transpose() * J_tmp_;

        qd_.noalias() = J_pseudo_inverse_ * yd_;

        // Support for a maximum step, e.g., when used as real-time, interactive IK
        if (GetNumberOfMaxIterations() == 1 && parameters_.MaxStep != 0.0)
        {
            ScaleToStepSize(qd_);
            q_ -= qd_;
        }
        // Line search
        else
        {
            for (int ai = 0; ai < alpha_space_.size(); ++ai)
            {
                steplength_ = alpha_space_(ai);
                Eigen::VectorXd q_tmp = q_ - steplength_ * qd_;
                prob_->Update(q_tmp);
                error_ = prob_->GetScalarCost();

                if (error_ < error_prev_)
                {
                    q_ = q_tmp;
                    qd_ *= steplength_;
                    break;
                }
            }
        }

        // Step tolerance parameter
        step_ = qd_.squaredNorm();

        // Gradient tolerance
        stop_ = cost_jacobian_.norm();

        // Debug output
        if (debug_) PrintDebug(i);

        // Check step tolerance
        if (step_ < parameters_.StepToleranceConvergenceThreshold)
        {
            prob_->termination_criterion = TerminationCriterion::StepTolerance;
            break;
        }

        // Check gradient tolerance (convergence)
        if (stop_ < parameters_.GradientToleranceConvergenceThreshold)
        {
            prob_->termination_criterion = TerminationCriterion::GradientTolerance;
            break;
        }

        // Adapt regularization based on step-length
        if (GetNumberOfMaxIterations() == 1 && parameters_.MaxStep == 0.0 && steplength_ > th_stepdec_)
        {
            DecreaseRegularization();
        }
        if (GetNumberOfMaxIterations() == 1 && parameters_.MaxStep == 0.0 && steplength_ <= th_stepinc_)
        {
            IncreaseRegularization();
            if (lambda_ == regmax_)
            {
                prob_->termination_criterion = TerminationCriterion::Divergence;
                break;
            }
        }
    }

    // Check if we ran out of iterations
    if (i == GetNumberOfMaxIterations())
    {
        prob_->termination_criterion = TerminationCriterion::IterationLimit;
    }

    if (debug_)
    {
        switch (prob_->termination_criterion)
        {
            case TerminationCriterion::GradientTolerance:
                HIGHLIGHT_NAMED("IKSolver", "Reached convergence (" << std::scientific << stop_ << " < " << parameters_.GradientToleranceConvergenceThreshold << ")");
                break;
            case TerminationCriterion::FunctionTolerance:
                HIGHLIGHT_NAMED("IKSolver", "Reached absolute function tolerance (" << std::scientific << error_ << " < " << parameters_.Tolerance << ")");
                break;
            case TerminationCriterion::StepTolerance:
                HIGHLIGHT_NAMED("IKSolver", "Reached step tolerance (" << std::scientific << step_ << " < " << parameters_.StepToleranceConvergenceThreshold << ")");
                break;
            case TerminationCriterion::Divergence:
                WARNING_NAMED("IKSolver", "Regularization exceeds maximum regularization: " << lambda_ << " > " << regmax_);
                break;
            case TerminationCriterion::IterationLimit:
                HIGHLIGHT_NAMED("IKSolver", "Reached iteration limit.");
                break;

            default:
                break;
        }
    }

    solution.resize(1, prob_->N);
    solution.row(0) = q_.transpose();
    planning_time_ = timer.GetDuration();
}

void IKSolver::ScaleToStepSize(Eigen::VectorXdRef xd)
{
    const double max_vel = xd.cwiseAbs().maxCoeff();
    if (max_vel > parameters_.MaxStep)
    {
        xd = xd * parameters_.MaxStep / max_vel;
    }
}
}  // namespace exotica
