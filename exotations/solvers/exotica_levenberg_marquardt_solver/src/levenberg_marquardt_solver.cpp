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

#include "exotica_levenberg_marquardt_solver/levenberg_marquardt_solver.h"

REGISTER_MOTIONSOLVER_TYPE("LevenbergMarquardtSolverSolver", exotica::LevenbergMarquardtSolver)

namespace exotica
{
void LevenbergMarquardtSolver::Instantiate(LevenbergMarquardtSolverInitializer& init) { parameters_ = init; }
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
}

void LevenbergMarquardtSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) ThrowNamed("Solver has not been initialized!");

    const Eigen::VectorXd q0 = prob_->ApplyStartState();

    if (prob_->N != q0.rows()) ThrowNamed("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    solution.resize(1, prob_->N);

    lambda_ = parameters_.damping;  // initial damping

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prob_->cost.jacobian.cols(), prob_->cost.jacobian.cols());
    Eigen::MatrixXd jacobian;

    Eigen::VectorXd q = q0;
    double error = std::numeric_limits<double>::infinity();
    double error_prev = std::numeric_limits<double>::infinity();
    Eigen::VectorXd yd;
    Eigen::VectorXd qd;
    for (size_t i = 0; i < GetNumberOfMaxIterations(); i++)
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
        if (parameters_.scale_problem == "none")
        {
            M = I;
        }
        else if (parameters_.scale_problem == "Jacobian")
        {
            M = (jacobian.transpose() * jacobian).diagonal().asDiagonal();
        }
        else
        {
            throw std::runtime_error("no ScaleProblem of type " + parameters_.scale_problem);
        }

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
        qd = (jacobian.transpose() * jacobian + lambda_ * M).completeOrthogonalDecomposition().solve(jacobian.transpose() * yd);
#else
        qd = (jacobian.transpose() * jacobian + lambda_ * M).colPivHouseholderQr().solve(jacobian.transpose() * yd);
#endif

        if (parameters_.alpha.size() == 1)
        {
            q -= qd * parameters_.alpha[0];
        }
        else
        {
            q -= qd.cwiseProduct(parameters_.alpha);
        }

        if (qd.norm() < parameters_.convergence)
        {
            if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "Reached convergence (" << qd.norm() << " < " << parameters_.convergence << ")");
            break;
        }
    }

    solution.row(0) = q;

    planning_time_ = timer.GetDuration();
}
}
