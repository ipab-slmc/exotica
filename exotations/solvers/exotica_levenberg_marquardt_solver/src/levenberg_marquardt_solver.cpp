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
    if (parameters_.Alpha.size() > 1 && parameters_.Alpha.size() != this->problem_->N)
    {
        ThrowNamed("Wrong alpha dimension: alpha(" << parameters_.Alpha.size() << ") != states(" << this->problem_->N << ")")
    }
}

void LevenbergMarquardtSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) ThrowNamed("Solver has not been initialized!");

    const Eigen::VectorXd q0 = prob_->ApplyStartState();

    if (prob_->N != q0.rows()) ThrowNamed("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    solution.resize(1, prob_->N);

    lambda_ = parameters_.Damping;  // initial damping

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prob_->cost.jacobian.cols(), prob_->cost.jacobian.cols());
    Eigen::MatrixXd jacobian;

    Eigen::VectorXd q = q0;
    double error = std::numeric_limits<double>::infinity();
    double error_prev = std::numeric_limits<double>::infinity();
    Eigen::VectorXd yd;
    Eigen::VectorXd qd;
    for (int i = 0; i < GetNumberOfMaxIterations(); ++i)
    {
        prob_->Update(q);

        yd = prob_->cost.S * prob_->cost.ydiff;

        // weighted sum of squares
        error_prev = error;
        error = prob_->GetScalarCost();

        prob_->SetCostEvolution(i, error);

        jacobian = prob_->cost.S * prob_->cost.jacobian;

        // source: https://uk.mathworks.com/help/optim/ug/least-squares-model-fitting-algorithms.html, eq. 13

        if (i > 0)
        {
            if (error < error_prev)
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

        Eigen::MatrixXd M;
        if (parameters_.ScaleProblem == "none")
        {
            M = I;
        }
        else if (parameters_.ScaleProblem == "Jacobian")
        {
            M = (jacobian.transpose() * jacobian).diagonal().asDiagonal();
        }
        else
        {
            throw std::runtime_error("no ScaleProblem of type " + parameters_.ScaleProblem);
        }

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
        qd = (jacobian.transpose() * jacobian + lambda_ * M).completeOrthogonalDecomposition().solve(jacobian.transpose() * yd);
#else
        qd = (jacobian.transpose() * jacobian + lambda_ * M).colPivHouseholderQr().solve(jacobian.transpose() * yd);
#endif

        if (parameters_.Alpha.size() == 1)
        {
            q -= qd * parameters_.Alpha[0];
        }
        else
        {
            q -= qd.cwiseProduct(parameters_.Alpha);
        }

        if (qd.norm() < parameters_.Convergence)
        {
            if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "Reached convergence (" << qd.norm() << " < " << parameters_.Convergence << ")");
            break;
        }
    }

    solution.row(0) = q;

    planning_time_ = timer.GetDuration();
}
}  // namespace exotica
