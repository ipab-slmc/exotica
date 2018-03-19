/*
 *  Created on: 13 Mar 2018
 *      Author: Wolfgang Merkt, Vladimir Ivan
 *
 *  This code is based on algorithm developed by Marc Toussaint
 *  M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
 *  http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
 *  Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html
 * 
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

#include "aico/BayesianIK.h"

REGISTER_MOTIONSOLVER_TYPE("BayesianIK", exotica::BayesianIK)

namespace exotica
{
void BayesianIK::Instantiate(BayesianIKInitializer& init)
{
    std::string mode = init.SweepMode;
    if (mode == "Forwardly")
        sweepMode = smForwardly;
    else if (mode == "Symmetric")
        sweepMode = smSymmetric;
    else if (mode == "LocalGaussNewton")
        sweepMode = smLocalGaussNewton;
    else if (mode == "LocalGaussNewtonDamped")
        sweepMode = smLocalGaussNewtonDamped;
    else
    {
        throw_named("Unknown sweep mode '" << init.SweepMode << "'");
    }
    setNumberOfMaxIterations(init.MaxIterations);
    max_backtrack_iterations = init.MaxBacktrackIterations;
    minimum_step_tolerance = init.MinStep;
    step_tolerance = init.StepTolerance;
    function_tolerance = init.FunctionTolerance;
    damping_init = init.Damping;
    useBwdMsg = init.UseBackwardMessage;
}

BayesianIK::BayesianIK()
    : damping(0.01),
      minimum_step_tolerance(1e-5),
      step_tolerance(1e-5),
      function_tolerance(1e-5),
      max_backtrack_iterations(10),
      useBwdMsg(false),
      bwdMsg_v(),
      bwdMsg_Vinv(),
      s(),
      Sinv(),
      v(),
      Vinv(),
      r(),
      R(),
      rhat(),
      b(),
      Binv(),
      q(),
      qhat(),
      s_old(),
      Sinv_old(),
      v_old(),
      Vinv_old(),
      r_old(),
      R_old(),
      rhat_old(),
      b_old(),
      Binv_old(),
      q_old(),
      qhat_old(),
      dampingReference(),
      cost(0.0),
      cost_old(0.0),
      cost_prev(std::numeric_limits<double>::max()),
      b_step(0.0),
      Winv(),
      sweep(0),
      sweepMode(smSymmetric),
      W(),
      updateCount(0),
      damping_init(100.0)
// q_stat()
{
}

BayesianIK::~BayesianIK() {}
void BayesianIK::specifyProblem(PlanningProblem_ptr problem)
{
    if (problem->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        throw_named("This solver can't use problem of type '" << problem->type() << "'!");
    }
    MotionSolver::specifyProblem(problem);
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(problem);

    initMessages();
}

void BayesianIK::Solve(Eigen::MatrixXd& solution)
{
    prob_->resetCostEvolution(getNumberOfMaxIterations() + 1);
    prob_->terminationCriterion = TerminationCriterion::NotStarted;
    planning_time_ = -1;

    Eigen::VectorXd q0 = prob_->applyStartState();

    Timer timer;
    if (debug_) ROS_WARN_STREAM("BayesianIK: Setting up the solver");
    updateCount = 0;
    damping = damping_init;
    double d;
    iterationCount = -1;
    initTrajectory(q0);
    if (debug_) ROS_WARN_STREAM("BayesianIK: Solving");

    // Reset sweep and iteration count
    sweep = 0;
    iterationCount = 0;
    while (iterationCount < getNumberOfMaxIterations())
    {
        // Check whether user interrupted (Ctrl+C)
        if (Server::isRos() && !ros::ok())
        {
            if (debug_) HIGHLIGHT("Solving cancelled by user");
            prob_->terminationCriterion = TerminationCriterion::UserDefined;
            break;
        }

        d = step();
        if (d < 0)
        {
            throw_named("Negative step size!");
        }

        // 0. Check maximum backtrack iterations
        if (sweep >= max_backtrack_iterations)
        {
            if (debug_) HIGHLIGHT("Maximum backtrack iterations reached, exiting.");
            prob_->terminationCriterion = TerminationCriterion::BacktrackIterationLimit;
            break;
        }

        // Check stopping criteria
        if (iterationCount > 1)
        {
            // Check convergence if
            //    a) damping is on and the iteration has concluded (the sweep improved the cost)
            //    b) damping is off [each sweep equals one iteration]
            if (damping && sweepImprovedCost || !damping)
            {
                // 1. Check step tolerance
                // || x_t-x_t-1 || <= stepTolerance * max(1, || x_t ||)
                // TODO(#257): TODO(#256): move to Eigen::MatrixXd to make this easier to compute, in the meantime use old check
                //
                // TODO(#256): OLD TOLERANCE CHECK - TODO REMOVE
                if (d < minimum_step_tolerance)
                {
                    if (debug_) HIGHLIGHT("Satisfied tolerance\titer=" << iterationCount << "\td=" << d << "\tminimum_step_tolerance=" << minimum_step_tolerance);
                    prob_->terminationCriterion = TerminationCriterion::StepTolerance;
                    break;
                }

                // 2. Check function tolerance
                // (f_t-1 - f_t) <= functionTolerance * max(1, abs(f_t))
                if ((cost_prev - cost) <= function_tolerance * std::max(1.0, std::abs(cost)))
                {
                    if (debug_) HIGHLIGHT("Function tolerance achieved: " << (cost_prev - cost) << " <= " << function_tolerance * std::max(1.0, std::abs(cost)));
                    prob_->terminationCriterion = TerminationCriterion::FunctionTolerance;
                    break;
                }
                cost_prev = cost;
            }
        }
    }

    // Check whether maximum iteration count was reached
    if (iterationCount == getNumberOfMaxIterations())
    {
        HIGHLIGHT("Maximum iterations reached");
        prob_->terminationCriterion = TerminationCriterion::IterationLimit;
    }

    solution.resize(1, prob_->N);
    solution.row(0) = q;
    planning_time_ = timer.getDuration();
}

void BayesianIK::initMessages()
{
    if (prob_ == nullptr) throw_named("Problem definition is a NULL pointer!");

    if (prob_->N < 1)
    {
        throw_named("State dimension is too small: n=" << prob_->N);
    }

    s = Eigen::VectorXd::Zero(prob_->N);
    Sinv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    v = Eigen::VectorXd::Zero(prob_->N);
    Vinv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    // if (useBwdMsg)
    // {
    //     if (bwdMsg_v.rows() == prob_->N && bwdMsg_Vinv.rows() == prob_->N && bwdMsg_Vinv.cols() == prob_->N)
    //     {
    //         v[prob_->getT() - 1] = bwdMsg_v;
    //         Vinv[prob_->getT() - 1] = bwdMsg_Vinv;
    //     }
    //     else
    //     {
    //         useBwdMsg = false;
    //         WARNING("Backward message initialisation skipped, matrices have incorrect dimensions.");
    //     }
    // }
    b = Eigen::VectorXd::Zero(prob_->N);
    dampingReference = Eigen::VectorXd::Zero(prob_->N);
    Binv = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    // Binv[0].setIdentity();
    // Binv[0] = Binv[0] * 1e10;
    r = Eigen::VectorXd::Zero(prob_->N);
    R = Eigen::MatrixXd::Zero(prob_->N, prob_->N);
    rhat = 0;
    qhat = Eigen::VectorXd::Zero(prob_->N);
    linSolverTmp.resize(prob_->N, prob_->N);
    {
        q = b;
        if (prob_->W.rows() != prob_->N)
        {
            throw_named(prob_->W.rows() << "!=" << prob_->N);
        }
    }
    {
        // Set constant W,Win,H,Hinv
        W = prob_->W;
        Winv = W.inverse();
    }
}

void BayesianIK::initTrajectory(const Eigen::VectorXd& q_init)
{
    qhat = q_init;
    q = q_init;
    dampingReference = q_init;
    b = q_init;
    s = q_init;
    v = q_init;
    Sinv.setZero();
    Sinv.diagonal().setConstant(damping);
    Vinv.setZero();
    Vinv.diagonal().setConstant(damping);

    // Compute task message reference
    updateTaskMessage(b, 0.0);

    cost = evaluateTrajectory(b, true);  // The problem will be updated via updateTaskMessage, i.e. do not update on this roll-out
    cost_prev = cost;
    prob_->setCostEvolution(0, cost);
    if (cost < 0) throw_named("Invalid cost! " << cost);
    if (debug_) HIGHLIGHT("Initial cost, updates: " << updateCount << ", cost: " << cost);
    rememberOldState();
}

void BayesianIK::updateFwdMessage()
{
    Eigen::MatrixXd barS(prob_->N, prob_->N), St;
    inverseSymPosDef(barS, Sinv + R);
    s = barS * (Sinv * s + r);
    St = Winv + barS;
    inverseSymPosDef(Sinv, St);
}

void BayesianIK::updateBwdMessage()
{
    Eigen::MatrixXd barV(prob_->N, prob_->N), Vt;
    // if (t < prob_->getT() - 1)
    // {
    //     inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]);
    //     v[t] = barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]);
    //     Vt = Winv + barV;
    //     inverseSymPosDef(Vinv[t], Vt);
    // }
    if (true)
    {
        if (!useBwdMsg)
        {
            v = b;
            Vinv.diagonal().setConstant(1);
        }
        else
        {
            v = bwdMsg_v;
            Vinv = bwdMsg_Vinv;
        }
    }
}

void BayesianIK::updateTaskMessage(const Eigen::Ref<const Eigen::VectorXd>& qhat_t, double minimum_step_tolerance,
                                   double maxStepSize)
{
    Eigen::VectorXd diff = qhat_t - qhat;
    if ((diff.array().abs().maxCoeff() < minimum_step_tolerance)) return;
    double nrm = diff.norm();
    if (maxStepSize > 0. && nrm > maxStepSize)
    {
        qhat += diff * (maxStepSize / nrm);
    }
    else
    {
        qhat = qhat_t;
    }

    prob_->Update(qhat);
    updateCount++;
    double c = getTaskCosts();
    // q_stat.addw(c > 0 ? 1.0 / (1.0 + c) : 1.0, qhat_t);
}

double BayesianIK::getTaskCosts()
{
    double C = 0;
    Eigen::MatrixXd Jt;
    double prec;
    rhat = 0;
    R.setZero();
    r.setZero();
    for (int i = 0; i < prob_->getTasks().size(); i++)
    {
        prec = prob_->Cost.Rho(i);
        if (prec > 0)
        {
            int start = prob_->Cost.Indexing[i].StartJ;
            int len = prob_->Cost.Indexing[i].LengthJ;
            Jt = prob_->Cost.J.middleRows(start, len).transpose();
            C += prec * (prob_->Cost.ydiff.segment(start, len)).squaredNorm();
            R += prec * Jt * prob_->Cost.J.middleRows(start, len);
            r += prec * Jt * (-prob_->Cost.ydiff.segment(start, len) + prob_->Cost.J.middleRows(start, len) * qhat);
            rhat += prec * (-prob_->Cost.ydiff.segment(start, len) + prob_->Cost.J.middleRows(start, len) * qhat).squaredNorm();
        }
    }
    return C;
}

void BayesianIK::updateTimeStep(bool updateFwd, bool updateBwd,
                                int maxRelocationIterations, double minimum_step_tolerance, bool forceRelocation,
                                double maxStepSize)
{
    if (updateFwd) updateFwdMessage();
    if (updateBwd) updateBwdMessage();

    if (damping)
    {
        Binv = Sinv + Vinv + R + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
        AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r + damping * dampingReference, linSolverTmp, prob_->N);
    }
    else
    {
        Binv = Sinv + Vinv + R;
        AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r, linSolverTmp, prob_->N);
    }

    for (int k = 0; k < maxRelocationIterations && !(Server::isRos() && !ros::ok()); k++)
    {
        if (!((!k && forceRelocation) || (b - qhat).array().abs().maxCoeff() > minimum_step_tolerance)) break;

        updateTaskMessage(b, 0., maxStepSize);

        //optional reUpdate fwd or bwd message (if the Dynamics might have changed...)
        if (updateFwd) updateFwdMessage();
        if (updateBwd) updateBwdMessage();

        if (damping)
        {
            Binv = Sinv + Vinv + R + Eigen::MatrixXd::Identity(prob_->N, prob_->N) * damping;
            AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r + damping * dampingReference, linSolverTmp, prob_->N);
        }
        else
        {
            Binv = Sinv + Vinv + R;
            AinvBSymPosDef(b, Binv, Sinv * s + Vinv * v + r, linSolverTmp, prob_->N);
        }
    }
}

void BayesianIK::updateTimeStepGaussNewton(bool updateFwd,
                                           bool updateBwd, int maxRelocationIterations, double minimum_step_tolerance,
                                           double maxStepSize)
{
    // TODO: implement updateTimeStepGaussNewton
    throw_named("Not implemented yet!");
}

double BayesianIK::evaluateTrajectory(const Eigen::VectorXd& x, bool skipUpdate)
{
    if (debug_) ROS_WARN_STREAM("Evaluating, iteration " << iterationCount << ", sweep " << sweep);
    q = x;

    // Perform update / roll-out
    if (!skipUpdate)
    {
        updateCount++;
        prob_->Update(q);
    }

    // Task cost
    return prob_->getScalarCost();
}

double BayesianIK::step()
{
    rememberOldState();
    switch (sweepMode)
    {
        //NOTE: the dependence on (Sweep?..:..) could perhaps be replaced by (DampingReference.N?..:..)
        case smForwardly:
            // for (t = 1; t < prob_->getT(); t++)
            // {
            updateTimeStep(true, false, 1, minimum_step_tolerance, !iterationCount, 1.);  //relocate once on fwd Sweep
            // }
            // for (t = prob_->getT() - 2; t > 0; t--)
            // {
            updateTimeStep(false, true, 0, minimum_step_tolerance, false, 1.);  //...not on bwd Sweep
            // }
            break;
        case smSymmetric:
            // ROS_WARN_STREAM("Updating forward, iterationCount "<<iterationCount);
            // for (t = 1; t < prob_->getT(); t++)
            // {
            updateTimeStep(true, false, 1, minimum_step_tolerance, !iterationCount, 1.);  //relocate once on fwd & bwd Sweep
            // }
            // ROS_WARN_STREAM("Updating backward, iterationCount "<<iterationCount);
            // for (t = prob_->getT() - 2; t > 0; t--)
            // {
            updateTimeStep(false, true, (iterationCount ? 1 : 0), minimum_step_tolerance, false, 1.);
            // }
            break;
        case smLocalGaussNewton:
            // for (t = 1; t < prob_->getT(); t++)
            // {
            //     updateTimeStep(t, true, false, (iterationCount ? 5 : 1), minimum_step_tolerance, !iterationCount, 1.);  //relocate iteratively on
            // }
            // for (t = prob_->getT() - 2; t > 0; t--)
            // {
            //     updateTimeStep(t, false, true, (iterationCount ? 5 : 0), minimum_step_tolerance, false, 1.);  //...fwd & bwd Sweep
            // }
            break;
        case smLocalGaussNewtonDamped:
            // for (t = 1; t < prob_->getT(); t++)
            // {
            //     updateTimeStepGaussNewton(t, true, false, (iterationCount ? 5 : 1),
            //                               minimum_step_tolerance, 1.);  //GaussNewton in fwd & bwd Sweep
            // }
            // for (t = prob_->getT() - 2; t > 0; t--)
            // {
            //     updateTimeStep(t, false, true, (iterationCount ? 5 : 0), minimum_step_tolerance, false, 1.);
            // }
            break;
        default:
            throw_named("non-existing Sweep mode");
    }
    b_step = std::max((b_old - b).array().abs().maxCoeff(), 0.0);
    dampingReference = b;
    // q is set inside of evaluateTrajectory() function
    cost = evaluateTrajectory(b);
    if (debug_) HIGHLIGHT("Iteration: " << iterationCount << ", Sweep: " << sweep << ", updates: " << updateCount << ", cost: " << cost << " (dq=" << b_step << ", damping=" << damping << ")");
    if (cost < 0) return -1.0;
    bestSweep = sweep;

    // If damping (similar to line-search) is being used, consider reverting this step
    if (damping) perhapsUndoStep();

    sweep++;
    if (sweepImprovedCost)
    {
        // HIGHLIGHT("Sweep improved cost, increasing iteration count and resetting sweep count");
        iterationCount++;
        sweep = 0;
        prob_->setCostEvolution(iterationCount, cost);
    }

    return b_step;
}

void BayesianIK::rememberOldState()
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
    cost_old = cost;

    bestSweep_old = bestSweep;
    b_step_old = b_step;
}

void BayesianIK::perhapsUndoStep()
{
    if (cost > cost_old)
    {
        sweepImprovedCost = false;
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
        cost = cost_old;
        dampingReference = b_old;
        bestSweep = bestSweep_old;
        b_step = b_step_old;
        if (debug_) HIGHLIGHT("Reverting to previous line-search step (" << bestSweep << ")");
    }
    else
    {
        sweepImprovedCost = true;
        damping /= 5.;
    }
}
} /* namespace exotica */
