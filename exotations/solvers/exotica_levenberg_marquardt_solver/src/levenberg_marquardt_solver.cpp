//
// Copyright 2018, University of Edinburgh
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in
//   all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "exotica_levenberg_marquardt_solver/levenberg_marquardt_solver.h"

REGISTER_MOTIONSOLVER_TYPE("LevenbergMarquardtSolverSolver", exotica::LevenbergMarquardtSolver)

namespace exotica
{
void LevenbergMarquardtSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        ThrowNamed("This LevenbergMarquardtSolver can't solve problem of type '" << pointer->type() << "'!");
    }

    MotionSolver::SpecifyProblem(pointer);

    // generic problem
    problem_ = pointer;

    // specific problem
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);

    // check dimension of alpha
    if (parameters_.Alpha.size() > 1 && parameters_.Alpha.size() != prob_->N)
    {
        ThrowNamed("Wrong alpha dimension: alpha(" << parameters_.Alpha.size() << ") != states(" << prob_->N << ")")
    }

    // Allocate variables
    JT_times_J_.resize(prob_->N, prob_->N);
    q_.resize(prob_->N);
    qd_.resize(prob_->N);
    yd_.resize(prob_->cost.S.rows());
    cost_jacobian_.resize(prob_->cost.S.rows(), prob_->N);

    if (parameters_.ScaleProblem == "none")
    {
        M_.setIdentity(prob_->N, prob_->N);
    }
    else if (parameters_.ScaleProblem == "Jacobian")
    {
        M_.setZero(prob_->N, prob_->N);
    }
    else
    {
        throw std::runtime_error("No ScaleProblem of type " + parameters_.ScaleProblem);
    }
}

void LevenbergMarquardtSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");

    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);
    Timer timer;

    q_ = prob_->ApplyStartState();
    if (prob_->N != q_.rows()) ThrowNamed("Wrong size q0 size=" << q_.rows() << ", required size=" << prob_->N);

    solution.resize(1, prob_->N);

    lambda_ = parameters_.Damping;  // Reset initial damping
    for (int i = 0; i < GetNumberOfMaxIterations(); ++i)
    {
        prob_->Update(q_);

        yd_.noalias() = prob_->cost.S * prob_->cost.ydiff;

        // weighted sum of squares
        error_prev_ = error_;
        error_ = prob_->GetScalarCost();

        prob_->SetCostEvolution(i, error_);

        cost_jacobian_.noalias() = prob_->cost.S * prob_->cost.jacobian;

        // source: https://uk.mathworks.com/help/optim/ug/least-squares-model-fitting-algorithms.html, eq. 13
        if (i > 0)
        {
            if (error_ < error_prev_)
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

        if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "damping: " << lambda_);

        if (parameters_.ScaleProblem == "Jacobian")
        {
            M_.diagonal().noalias() = (cost_jacobian_.transpose() * cost_jacobian_).diagonal();
        }

        JT_times_J_.noalias() = cost_jacobian_.transpose() * cost_jacobian_;
        JT_times_J_ += lambda_ * M_;

        llt_.compute(JT_times_J_);
        if (llt_.info() != Eigen::Success)
        {
            ThrowPretty("Error during matrix decomposition of J^T * J (lambda=" << lambda_ << "):\n"
                                                                                << JT_times_J_);
        }
        qd_.noalias() = cost_jacobian_.transpose() * yd_;
        llt_.solveInPlace(qd_);

        if (parameters_.Alpha.size() == 1)
        {
            q_ -= qd_ * parameters_.Alpha[0];
        }
        else
        {
            q_ -= qd_.cwiseProduct(parameters_.Alpha);
        }

        if (qd_.norm() < parameters_.Convergence)
        {
            if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "Reached convergence (" << qd_.norm() << " < " << parameters_.Convergence << ")");
            break;
        }
    }

    solution.row(0) = q_;
    planning_time_ = timer.GetDuration();
}
}  // namespace exotica
