/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
 *
 *  This code is based on algorithm developed by Marc Toussaint
 *  M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
 *  http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
 *  Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html
 * 
 * Copyright (c) 2016, University Of Edinburgh 
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

/** \file AICOsolver.h
 \brief Approximate Inference Control */

#include "aico/AICOsolver.h"

REGISTER_MOTIONSOLVER_TYPE("AICOsolver", exotica::AICOsolver)

namespace exotica
{
void AICOsolver::Instantiate(AICOsolverInitializer& init)
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
    max_iterations = init.MaxIterations;
    tolerance = init.Tolerance;
    damping_init = init.Damping;
    useBwdMsg = init.UseBackwardMessage;
    dynamic = init.Dynamic;
}

void AICOsolver::saveCosts(std::string file_name)
{
    std::ofstream myfile;
    myfile.open(file_name.c_str());
    myfile << "Control Task";
    for (int t = 0; t < prob_->getT(); t++)
    {
        myfile << "\n"
               << costControl(t);
        myfile << " " << costTask(t);
    }
    myfile.close();
}

AICOsolver::AICOsolver()
    : damping(0.01), tolerance(1e-2), max_iterations(100), useBwdMsg(false), bwdMsg_v(), bwdMsg_Vinv(), s(), Sinv(), v(), Vinv(), r(), R(), rhat(), b(), Binv(), q(), qhat(), s_old(), Sinv_old(), v_old(), Vinv_old(), r_old(), R_old(), rhat_old(), b_old(), Binv_old(), q_old(), qhat_old(), dampingReference(), cost(0.0), cost_old(0.0), b_step(0.0), A(), tA(), Ainv(), invtA(), a(), B(), tB(), Winv(), Hinv(), Q(), sweep(0), sweepMode(0), W(), H(), dynamic(false), n(0), updateCount(0), damping_init(0.0), preupdateTrajectory_(false), q_stat()
{
}

void AICOsolver::getStats(std::vector<SinglePassMeanCoviariance>& q_stat_)
{
    q_stat_ = q_stat;
}

AICOsolver::~AICOsolver()
{
    // If this is not empty, your code is bad and you should feel bad!
    // Whoop whoop whoop whoop ...
}

void AICOsolver::specifyProblem(PlanningProblem_ptr problem)
{
    if (problem->type() != "exotica::UnconstrainedTimeIndexedProblem")
    {
        throw_named("This solver can't use problem of type '" << problem->type() << "'!");
    }
    MotionSolver::specifyProblem(problem);
    prob_ = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(problem);

    initMessages();
}

void AICOsolver::Solve(Eigen::MatrixXd& solution)
{
    // Check if the trajectory length has changed, if so update the messages.
    if (prob_->getT() != lastT) initMessages();

    prob_->resetCostEvolution(max_iterations);

    Eigen::VectorXd q0 = prob_->applyStartState();
    std::vector<Eigen::VectorXd> q_init = prob_->getInitialTrajectory();

    // If the initial value of the initial trajectory does not equal the start
    // state, assume that no initial guess is provided and fill the trajectory
    // with the start state
    if (!(q0 - q_init[0]).isMuchSmallerThan(1e-6))
    {
        q_init.resize(prob_->getT(), Eigen::VectorXd::Zero(q0.rows()));
        for (int i = 0; i < q_init.size(); i++) q_init[i] = q0;
    }
    else
    {
        HIGHLIGHT("AICO::Solve called with initial trajectory guess");
    }

    prob_->preupdate();
    prob_->setStartState(q_init[0]);
    prob_->applyStartState();

    Timer timer;
    if (debug_) ROS_WARN_STREAM("AICO: Setting up the solver");
    updateCount = 0;
    sweep = -1;
    damping = damping_init;
    double d;
    if (!(prob_->getT() > 0))
    {
        throw_named("Problem has not been initialized properly: T=0!");
    }
    initTrajectory(q_init);
    sweep = 0;
    if (debug_) ROS_WARN_STREAM("AICO: Solving");
    for (int k = 0; k < max_iterations && !(Server::isRos() && !ros::ok()); k++)
    {
        d = step();
        if (d < 0)
        {
            throw_named("Negative step size!");
        }
        if (k && d < tolerance) break;
    }
    Eigen::MatrixXd sol(prob_->getT(), n);
    for (int tt = 0; tt < prob_->getT(); tt++)
    {
        sol.row(tt) = q[tt];
    }
    solution = sol;
    planning_time_ = timer.getDuration();
}

void AICOsolver::initMessages()
{
    if (prob_ == nullptr) throw_named("Problem definition is a NULL pointer!");
    // TODO: Issue #4
    n = prob_->N;
    int n2 = n / 2;
    if (dynamic)
    {
        if (n < 2)
        {
            throw_named("State dimension is too small: n=" << n);
        }
    }
    else
    {
        if (n < 1)
        {
            throw_named("State dimension is too small: n=" << n);
        }
    }
    if (prob_->getT() < 2)
    {
        throw_named("Number of time steps is too small: T=" << prob_->getT());
    }

    s.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    Sinv.assign(prob_->getT(), Eigen::MatrixXd::Zero(n, n));
    Sinv[0].diagonal().setConstant(1e10);
    v.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    Vinv.assign(prob_->getT(), Eigen::MatrixXd::Zero(n, n));
    if (useBwdMsg)
    {
        if (bwdMsg_v.rows() == n && bwdMsg_Vinv.rows() == n && bwdMsg_Vinv.cols() == n)
        {
            v[prob_->getT()] = bwdMsg_v;
            Vinv[prob_->getT()] = bwdMsg_Vinv;
        }
        else
        {
            useBwdMsg = false;
            WARNING("Backward message initialisation skipped, matrices have incorrect dimensions.");
        }
    }
    b.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    dampingReference.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    Binv.assign(prob_->getT(), Eigen::MatrixXd::Zero(n, n));
    Binv[0].setIdentity();
    Binv[0] = Binv[0] * 1e10;
    r.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    R.assign(prob_->getT(), Eigen::MatrixXd::Zero(n, n));
    rhat = Eigen::VectorXd::Zero(prob_->getT());
    qhat.assign(prob_->getT(), Eigen::VectorXd::Zero(n));
    linSolverTmp.resize(n, n);
    {
        if (dynamic)
        {
            q.resize(prob_->getT());
            for (int i = 0; i < q.size(); i++)
                q.at(i) = b.at(i).head(n2);
            if (prob_->W.rows() != n2)
            {
                throw_named(prob_->W.rows() << "!=" << n2);
            }
        }
        else
        {
            q = b;
            if (prob_->W.rows() != n)
            {
                throw_named(prob_->W.rows() << "!=" << n);
            }
        }
        // All the process parameters are assumed to be constant throughout the trajectory.
        // This is possible for a pseudo-dynamic system.
        // A dynamic system would have to update these based on forces acting on the system.
        Eigen::MatrixXd A_(n, n), B_(dynamic ? n2 : n, dynamic ? n2 : n), tB_,
            tA_, Ainv_, invtA_;
        Eigen::VectorXd a_(n);
        getProcess(A_, a_, B_);
        tB_ = B_.transpose();
        tA_ = A_.transpose();
        Ainv_ = A_.inverse();
        invtA_ = Ainv_.transpose();
        B.assign(prob_->getT(), B_);
        tB.assign(prob_->getT(), tB_);
        A.assign(prob_->getT(), A_);
        tA.assign(prob_->getT(), tA_);
        Ainv.assign(prob_->getT(), Ainv_);
        invtA.assign(prob_->getT(), invtA_);
        a.assign(prob_->getT(), a_);
    }
    {
        // Set constant W,Win,H,Hinv
        W.assign(prob_->getT(), prob_->W);
        Winv.assign(prob_->getT(), prob_->W.inverse());
        H.assign(prob_->getT(), prob_->H);
        Hinv.assign(prob_->getT(), prob_->H.inverse());
        Q.assign(prob_->getT(), prob_->Q);
    }

    costControl.resize(prob_->getT());
    costControl.setZero();
    costTask.resize(prob_->getT());
    costTask.setZero();

    q_stat.resize(prob_->getT());
    for (int t = 0; t < prob_->getT(); t++)
    {
        q_stat[t].resize(n);
    }

    // Set lastT to the problem T
    lastT = prob_->getT();
}

void AICOsolver::getProcess(Eigen::Ref<Eigen::MatrixXd> A_,
                            Eigen::Ref<Eigen::VectorXd> a_, Eigen::Ref<Eigen::MatrixXd> B_)
{
    if (!dynamic)
    {
        A_ = Eigen::MatrixXd::Identity(n, n);
        B_ = Eigen::MatrixXd::Identity(n, n);
        a_ = Eigen::VectorXd::Zero(n);
    }
    else
    {
        double tau = prob_->getTau();
        int n2 = n / 2;
        A_ = Eigen::MatrixXd::Identity(n, n);
        B_ = Eigen::MatrixXd::Zero(n, n2);
        a_ = Eigen::VectorXd::Zero(n);

        // Assuming pseudo dynamic process (M is identity matrix and F is zero vector)
        Eigen::MatrixXd Minv = Eigen::MatrixXd::Identity(n2, n2);
        Eigen::VectorXd F = Eigen::VectorXd::Zero(n2);

        A_.topRightCorner(n2, n2).diagonal().setConstant(tau);
        B_.topLeftCorner(n2, n2) = Minv * (tau * tau * 0.5);
        B_.bottomLeftCorner(n2, n2) = Minv * tau;
        a_.head(n2) = Minv * F * (tau * tau * 0.5);
        a_.tail(n2) = Minv * F * tau;
    }
}

void AICOsolver::initTrajectory(const std::vector<Eigen::VectorXd>& q_init)
{
    if (q_init.size() != prob_->getT())
    {
        throw_named("Incorrect number of timesteps provided!");
    }
    qhat[0] = q_init[0];
    dampingReference[0] = q_init[0];
    b[0] = q_init[0];
    s[0] = q_init[0];
    rememberOldState();
    int n2 = n / 2;
    b = q_init;
    for (int i = 0; i < q.size(); i++)
        q.at(i) = b.at(i).head(n2);
    s = b;
    for (int t = 1; t < prob_->getT(); t++)
    {
        Sinv.at(t).setZero();
        Sinv.at(t).diagonal().setConstant(damping);
    }
    v = b;
    for (int t = 0; t < prob_->getT(); t++)
    {
        Vinv.at(t).setZero();
        Vinv.at(t).diagonal().setConstant(damping);
    }
    dampingReference = b;
    for (int t = 0; t < prob_->getT(); t++)
    {
        // Compute task message reference
        updateTaskMessage(t, b.at(t), 0.0);
    }

    cost = evaluateTrajectory(b, true);
    if (cost < 0) throw_named("Invalid cost! " << cost);
    INFO("Initial cost(ctrl/task/total): " << costControl.sum() << "/" << costTask.sum() << "/" << cost << ", updates: " << updateCount);
    rememberOldState();
}

void AICOsolver::inverseSymPosDef(Eigen::Ref<Eigen::MatrixXd> Ainv_,
                                  const Eigen::Ref<const Eigen::MatrixXd>& A_)
{
    Ainv_ = A_;
    double* AA = Ainv_.data();
    integer info;
    integer nn = A_.rows();
    // Compute Cholesky
    dpotrf_((char*)"L", &nn, AA, &nn, &info);
    if (info != 0)
    {
        throw_named("Cholesky decomposition error: " << info << "\n"
                                                     << A_);
    }
    // Invert
    dpotri_((char*)"L", &nn, AA, &nn, &info);
    if (info != 0)
    {
        throw_named("Matrix inversion error: " << info);
    }
    Ainv_.triangularView<Eigen::Upper>() = Ainv_.transpose();
}

void AICOsolver::AinvBSymPosDef(Eigen::Ref<Eigen::VectorXd> x_,
                                const Eigen::Ref<const Eigen::MatrixXd>& A_,
                                const Eigen::Ref<const Eigen::VectorXd>& b_)
{
    integer n_ = n, m_ = 1;
    integer info;
    linSolverTmp = A_;
    x_ = b_;
    double* AA = linSolverTmp.data();
    double* xx = x_.data();
    dposv_((char*)"L", &n_, &m_, AA, &n_, xx, &n_, &info);
    if (info != 0)
    {
        throw_named("Linear solver error: " << info << "\nA:\n"
                                            << A_ << "\nb: " << b_.transpose() << "\nx: " << x_.transpose());
    }
}

void AICOsolver::updateFwdMessage(int t)
{
    Eigen::MatrixXd barS(n, n), St;
    if (dynamic)
    {
        inverseSymPosDef(barS, Sinv[t - 1] + R[t - 1]);
        St = Q[t - 1] + B[t - 1] * Hinv[t - 1] * tB[t - 1] + A[t - 1] * barS * tA[t - 1];
        s[t] = a[t - 1] + A[t - 1] * (barS * (Sinv[t - 1] * s[t - 1] + r[t - 1]));
        inverseSymPosDef(Sinv[t], St);
    }
    else
    {
        inverseSymPosDef(barS, Sinv[t - 1] + R[t - 1]);
        s[t] = barS * (Sinv[t - 1] * s[t - 1] + r[t - 1]);
        St = Winv[t - 1] + barS;
        inverseSymPosDef(Sinv[t], St);
    }
}

void AICOsolver::updateBwdMessage(int t)
{
    Eigen::MatrixXd barV(n, n), Vt;
    if (dynamic)
    {
        if (t < prob_->getT() - 1)
        {
            inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]);
            Vt = Ainv[t] * (Q[t] + B[t] * Hinv[t] * tB[t] + barV) * invtA[t];
            v[t] = Ainv[t] * (-a[t] + barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]));
            inverseSymPosDef(Vinv[t], Vt);
        }
        if (t == prob_->getT() - 1)
        {
            if (!useBwdMsg)
            {
                v[t] = b[t];
                Vinv[t].diagonal().setConstant(1e-4);
            }
            else
            {
                v[prob_->getT() - 1] = bwdMsg_v;
                Vinv[prob_->getT() - 1] = bwdMsg_Vinv;
            }
        }
    }
    else
    {
        if (t < prob_->getT() - 1)
        {
            inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]);
            v[t] = barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]);
            Vt = Winv[t] + barV;
            inverseSymPosDef(Vinv[t], Vt);
        }
        if (t == prob_->getT() - 1)
        {
            if (!useBwdMsg)
            {
                v[t] = b[t];
                Vinv[t].diagonal().setConstant(1e-0);
            }
            else
            {
                v[prob_->getT() - 1] = bwdMsg_v;
                Vinv[prob_->getT() - 1] = bwdMsg_Vinv;
            }
        }
    }
}

void AICOsolver::updateTaskMessage(int t,
                                   const Eigen::Ref<const Eigen::VectorXd>& qhat_t, double tolerance_,
                                   double maxStepSize)
{
    Eigen::VectorXd diff = qhat_t - qhat[t];
    if ((diff.array().abs().maxCoeff() < tolerance_)) return;
    double nrm = diff.norm();
    if (maxStepSize > 0. && nrm > maxStepSize)
    {
        qhat[t] += diff * (maxStepSize / nrm);
    }
    else
    {
        qhat[t] = qhat_t;
    }

    prob_->Update(dynamic ? qhat[t].head(n / 2) : qhat[t], t);
    updateCount++;
    double c = getTaskCosts(t);
    q_stat[t].addw(c > 0 ? 1.0 / (1.0 + c) : 1.0, qhat_t);

    // If using fully dynamic system, update Q, Hinv and process variables here.
}

double AICOsolver::getTaskCosts(int t)
{
    double C = 0;
    if (!dynamic)
    {
        Eigen::MatrixXd Jt;
        double prec;
        rhat[t] = 0;
        R[t].setZero();
        r[t].setZero();
        for (int i = 0; i < prob_->getTasks().size(); i++)
        {
            prec = prob_->Cost.Rho[t](i);
            if (prec > 0)
            {
                int start = prob_->Cost.Indexing[i].StartJ;
                int len = prob_->Cost.Indexing[i].LengthJ;
                Jt = prob_->J[t].middleRows(start, len).transpose();
                C += prec * (prob_->Cost.ydiff[t].segment(start, len)).squaredNorm();
                R[t] += prec * Jt * prob_->J[t].middleRows(start, len);
                r[t] += prec * Jt * (-prob_->Cost.ydiff[t].segment(start, len) + prob_->Cost.J[t].middleRows(start, len) * qhat[t]);
                rhat[t] += prec * (-prob_->Cost.ydiff[t].segment(start, len) + prob_->Cost.J[t].middleRows(start, len) * qhat[t]).squaredNorm();
            }
        }
    }
    else
    {
        Eigen::MatrixXd Jt;
        int n2 = n / 2;
        double prec;
        rhat[t] = 0;
        R[t].setZero();
        r[t].setZero();
        for (int i = 0; i < prob_->getTasks().size(); i++)
        {
            prec = prob_->Cost.Rho[t](i);
            if (prec > 0)
            {
                int start = prob_->Cost.Indexing[i].StartJ;
                int len = prob_->Cost.Indexing[i].LengthJ;
                Jt = prob_->Cost.J[t].middleRows(start, len).transpose();
                C += prec * (prob_->Cost.ydiff[t].segment(start, len)).squaredNorm();
                R[t].topLeftCorner(n2, n2) += prec * Jt * prob_->Cost.J[t].middleRows(start, len);
                r[t].head(n2) += prec * Jt * (-prob_->Cost.ydiff[t].segment(start, len) + prob_->Cost.J[t].middleRows(start, len) * qhat[t]);
                rhat[t] += prec * (-prob_->Cost.ydiff[t].segment(start, len) + prob_->Cost.J[t].middleRows(start, len) * qhat[t]).squaredNorm();
            }
        }
    }
    return prob_->ct * C;
}

void AICOsolver::updateTimeStep(int t, bool updateFwd, bool updateBwd,
                                int maxRelocationIterations, double tolerance_, bool forceRelocation,
                                double maxStepSize)
{
    if (updateFwd) updateFwdMessage(t);
    if (updateBwd) updateBwdMessage(t);

    if (damping)
    {
        Binv[t] = Sinv[t] + Vinv[t] + R[t] + Eigen::MatrixXd::Identity(n, n) * damping;
        AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t] + damping * dampingReference[t]);
    }
    else
    {
        Binv[t] = Sinv[t] + Vinv[t] + R[t];
        AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]);
    }

    for (int k = 0; k < maxRelocationIterations && !(Server::isRos() && !ros::ok()); k++)
    {
        if (!((!k && forceRelocation) || (b[t] - qhat[t]).array().abs().maxCoeff() > tolerance_)) break;

        updateTaskMessage(t, b.at(t), 0., maxStepSize);

        //optional reUpdate fwd or bwd message (if the Dynamics might have changed...)
        if (updateFwd) updateFwdMessage(t);
        if (updateBwd) updateBwdMessage(t);

        if (damping)
        {
            Binv[t] = Sinv[t] + Vinv[t] + R[t] + Eigen::MatrixXd::Identity(n, n) * damping;
            AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t] + damping * dampingReference[t]);
        }
        else
        {
            Binv[t] = Sinv[t] + Vinv[t] + R[t];
            AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]);
        }
    }
}

void AICOsolver::updateTimeStepGaussNewton(int t, bool updateFwd,
                                           bool updateBwd, int maxRelocationIterations, double tolerance,
                                           double maxStepSize)
{
    // TODO: implement updateTimeStepGaussNewton
    throw_named("Not implemented yet!");
}

double AICOsolver::evaluateTrajectory(const std::vector<Eigen::VectorXd>& x,
                                      bool skipUpdate)
{
    if (debug_) ROS_WARN_STREAM("Evaluating, sweep " << sweep);
    Timer timer;
    double dSet, dPre, dUpd, dCtrl, dTask;
    // double tau = prob_->tau;
    // double tau_1 = 1. / tau, tau_2 = tau_1 * tau_1;
    int n2 = n / 2;
    Eigen::VectorXd vv;

    if (dynamic)
    {
        for (int i = 0; i < q.size(); i++)
            q.at(i) = x.at(i).head(n2);
    }
    else
    {
        q = x;
    }
    dSet = timer.getDuration();
    if (preupdateTrajectory_)
    {
        if (debug_) ROS_WARN_STREAM("Pre-update, sweep " << sweep);
        for (int t = 0; t < prob_->getT(); t++)
        {
            if (Server::isRos() && !ros::ok()) return -1.0;
            updateCount++;
            prob_->Update(q[t], t);
        }
    }
    dPre = timer.getDuration();

    for (int t = 0; t < prob_->getT(); t++)
    {
        timer.reset();
        if (Server::isRos() && !ros::ok()) return -1.0;
        if (!skipUpdate)
        {
            updateCount++;
            prob_->Update(q[t], t);
        }
        dUpd += timer.getDuration();
        timer.reset();

        // Control cost
        if (!dynamic)
        {
            costControl(t) = prob_->getScalarTransitionCost(t);
        }
        else
        {
            throw_pretty("Not supported at the moment");
            // if (t < prob_->getT() - 1 && t > 0)
            // {
            //     // For fully dynamic system use: v=tau_2*M*(q[t+1]+q[t-1]-2.0*q[t])-F;
            //     vv = tau_2 * (q[t + 1] + q[t - 1] - 2.0 * q[t]);
            //     costControl(t) = vv.transpose() * H[t] * vv;
            // }
            // else if (t == 0)
            // {
            //     // For fully dynamic system use: v=tau_2*M*(q[t+1]+q[t])-F;
            //     vv = tau_2 * (q[t + 1] + q[t]);
            //     costControl(t) = vv.transpose() * H[t] * vv;
            // }
            // else
            //     costControl(t) = 0.0;
        }

        dCtrl += timer.getDuration();
        timer.reset();
        // Task cost
        costTask(t) = prob_->getScalarTaskCost(t);
        dTask += timer.getDuration();
    }

    return costControl.sum() + costTask.sum();
}

double AICOsolver::step()
{
    rememberOldState();
    int t;
    switch (sweepMode)
    {
        //NOTE: the dependence on (Sweep?..:..) could perhaps be replaced by (DampingReference.N?..:..)
        case smForwardly:
            for (t = 1; t < prob_->getT(); t++)
            {
                updateTimeStep(t, true, false, 1, tolerance, !sweep, 1.);  //relocate once on fwd Sweep
            }
            for (t = prob_->getT() - 2; t >= 0; t--)
            {
                updateTimeStep(t, false, true, 0, tolerance, false, 1.);  //...not on bwd Sweep
            }
            break;
        case smSymmetric:
            //ROS_WARN_STREAM("Updating forward, sweep "<<sweep);
            for (t = 1; t < prob_->getT(); t++)
            {
                updateTimeStep(t, true, false, 1, tolerance, !sweep, 1.);  //relocate once on fwd & bwd Sweep
            }
            //ROS_WARN_STREAM("Updating backward, sweep "<<sweep);
            for (t = prob_->getT() - 2; t >= 0; t--)
            {
                updateTimeStep(t, false, true, (sweep ? 1 : 0), tolerance, false, 1.);
            }
            break;
        case smLocalGaussNewton:
            for (t = 1; t < prob_->getT(); t++)
            {
                updateTimeStep(t, true, false, (sweep ? 5 : 1), tolerance, !sweep, 1.);  //relocate iteratively on
            }
            for (t = prob_->getT() - 2; t >= 0; t--)
            {
                updateTimeStep(t, false, true, (sweep ? 5 : 0), tolerance, false, 1.);  //...fwd & bwd Sweep
            }
            break;
        case smLocalGaussNewtonDamped:
            for (t = 1; t < prob_->getT(); t++)
            {
                updateTimeStepGaussNewton(t, true, false, (sweep ? 5 : 1),
                                          tolerance, 1.);  //GaussNewton in fwd & bwd Sweep
            }
            for (t = prob_->getT() - 2; t >= 0; t--)
            {
                updateTimeStep(t, false, true, (sweep ? 5 : 0), tolerance, false, 1.);
            }
            break;
        default:
            throw_named("non-existing Sweep mode");
    }
    b_step = 0.0;
    for (t = 0; t < b.size(); t++)
    {
        b_step = std::max((b_old[t] - b[t]).array().abs().maxCoeff(), b_step);
    }
    dampingReference = b;
    // q is set inside of evaluateTrajectory() function
    cost = evaluateTrajectory(b);
    prob_->setCostEvolution(sweep, cost);
    if (debug_) HIGHLIGHT("Sweep: " << sweep << ", updates: " << updateCount << ", cost(ctrl/task/total): " << costControl.sum() << "/" << costTask.sum() << "/" << cost << " (dq=" << b_step << ", damping=" << damping << ")");
    if (cost < 0) return -1.0;

    if (sweep && damping) perhapsUndoStep();
    sweep++;
    return b_step;
}

void AICOsolver::rememberOldState()
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
    costControl_old = costControl;
    costTask_old = costTask;
    dim_old = dim;
}

void AICOsolver::perhapsUndoStep()
{
    if (cost > cost_old)
    {
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
        costControl = costControl_old;
        costTask = costTask_old;
        dim = dim_old;
        if (preupdateTrajectory_)
        {
            for (int t = 0; t < prob_->getT(); t++)
            {
                updateCount++;
                prob_->Update(q[t], t);
            }
        }
        INFO("Reverting to previous step");
    }
    else
    {
        damping /= 5.;
    }
}

} /* namespace exotica */
