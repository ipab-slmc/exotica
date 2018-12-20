/*
 * Copyright (c) 2018, University of Edinburgh
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include <exotica_ik_solver/ik_solver.h>

REGISTER_MOTIONSOLVER_TYPE("IKSolver", exotica::IKSolver)

namespace exotica
{
IKSolver::IKSolver() = default;

IKSolver::~IKSolver() = default;

void IKSolver::Instantiate(IKSolverInitializer& init)
{
    parameters_ = init;
}

void IKSolver::specifyProblem(PlanningProblem_ptr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        throw_named("This IKSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);

    if (parameters_.C < 0 || parameters_.C >= 1.0)
        throw_named("C must be from interval <0, 1)!");
    C_ = Eigen::MatrixXd::Identity(prob_->Cost.JN, prob_->Cost.JN) * parameters_.C;
    W_ = prob_->W;

    if (parameters_.Alpha.size() != 1 && prob_->N != parameters_.Alpha.size())
        throw_named("Alpha must have length of 1 or N.");
}

void IKSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->resetCostEvolution(getNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) throw_named("Solver has not been initialized!");
    const Eigen::VectorXd q0 = prob_->applyStartState();

    if (prob_->N != q0.rows()) throw_named("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    Eigen::VectorXd qd, q_nominal;
    if (prob_->qNominal.rows() == prob_->N)
    {
        q_nominal = prob_->qNominal;
    }
    else
    {
        q_nominal = q0;
    }

    solution.resize(1, prob_->N);

    Eigen::VectorXd q = q0;
    double error = std::numeric_limits<double>::infinity();
    bool is_regularised = C_(0, 0) > 0.0;
    Eigen::VectorXd yd;
    Eigen::MatrixXd J;
    if (is_regularised)
    {
        yd = Eigen::VectorXd(prob_->JN + prob_->N);
        J = Eigen::MatrixXd(prob_->JN + prob_->N, prob_->N);
    }
    for (int i = 0; i < getNumberOfMaxIterations(); ++i)
    {
        prob_->Update(q);

        error = prob_->getScalarCost();

        prob_->setCostEvolution(i, error);

        if (error < parameters_.Tolerance)
        {
            if (debug_)
                HIGHLIGHT_NAMED("IKSolver", "Reached tolerance (" << error << " < " << parameters_.Tolerance << ")");
            break;
        }

        if (is_regularised)
        {
            yd.head(prob_->JN) = prob_->Cost.S * prob_->Cost.ydiff;
            yd.tail(prob_->N) = q - q_nominal;
            J.topRows(prob_->JN) = prob_->Cost.S * prob_->Cost.J * W_ * (1.0 - C_(0, 0));
            J.bottomRows(prob_->N) = C_;
        }
        else
        {
            yd = prob_->Cost.S * prob_->Cost.ydiff;
            J = prob_->Cost.S * prob_->Cost.J * W_;
        }

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
        qd = J.completeOrthogonalDecomposition().solve(yd);
#else
        qd = J.colPivHouseholderQr().solve(yd);
#endif

        ScaleToStepSize(qd);

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
            if (debug_)
                HIGHLIGHT_NAMED("IKSolver", "Reached convergence (" << qd.norm() << " < " << parameters_.Convergence
                                                                    << ")");
            break;
        }
    }

    solution.row(0) = q;

    planning_time_ = timer.getDuration();
}

void IKSolver::ScaleToStepSize(Eigen::VectorXdRef xd)
{
    double max_vel = xd.cwiseAbs().maxCoeff();
    if (max_vel > parameters_.MaxStep)
    {
        xd = xd * parameters_.MaxStep / max_vel;
    }
}
}
