/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#include <exotica/Problems/TimeIndexedProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedProblem", exotica::TimeIndexedProblem)

namespace exotica
{
TimeIndexedProblem::TimeIndexedProblem()
    : T(0), tau(0), W_rate(0)
{
    Flags = KIN_FK | KIN_J;
}

TimeIndexedProblem::~TimeIndexedProblem()
{
}

std::vector<double>& TimeIndexedProblem::getBounds()
{
    return bounds_;
}

void TimeIndexedProblem::Instantiate(TimeIndexedProblemInitializer& init)
{
    init_ = init;
    applyStartState(false);
    setT(init_.T);

    std::vector<std::string> jnts;
    scene_->getJointNames(jnts);

    bounds_.resize(jnts.size() * 2);
    std::map<std::string, std::vector<double>> joint_limits = scene_->getSolver().getUsedJointLimits();
    for (int i = 0; i < jnts.size(); i++)
    {
        bounds_[i] = joint_limits.at(jnts[i])[0];
        bounds_[i + jnts.size()] = joint_limits.at(jnts[i])[1];
    }

    useBounds = init.UseBounds;
}

void TimeIndexedProblem::reinitializeVariables()
{
    T = init_.T;
    if (T <= 2)
    {
        throw_named("Invalid number of timesteps: " << T);
    }
    tau = init_.Tau;
    W_rate = init_.Wrate;

    NumTasks = Tasks.size();
    PhiN = 0;
    JN = 0;
    TaskSpaceVector yref;
    for (int i = 0; i < NumTasks; i++)
    {
        appendVector(yref.map, Tasks[i]->getLieGroupIndices());
        PhiN += Tasks[i]->Length;
        JN += Tasks[i]->LengthJ;
    }

    N = scene_->getSolver().getNumControlledJoints();

    W = Eigen::MatrixXd::Identity(N, N) * W_rate;
    if (init_.W.rows() > 0)
    {
        if (init_.W.rows() == N)
        {
            W.diagonal() = init_.W * W_rate;
        }
        else
        {
            throw_named("W dimension mismatch! Expected " << N << ", got " << init_.W.rows());
        }
    }

    yref.setZero(PhiN);
    Phi.assign(T, yref);
    J.assign(T, Eigen::MatrixXd(JN, N));

    // Set initial trajectory
    InitialTrajectory.resize(T, scene_->getControlledState());

    Cost.initialize(init_.Cost, shared_from_this(), CostPhi);
    Inequality.initialize(init_.Inequality, shared_from_this(), InequalityPhi);
    Equality.initialize(init_.Equality, shared_from_this(), EqualityPhi);
    Cost.reinitializeVariables(T, shared_from_this(), CostPhi);
    Inequality.reinitializeVariables(T, shared_from_this(), InequalityPhi);
    Equality.reinitializeVariables(T, shared_from_this(), EqualityPhi);
}

void TimeIndexedProblem::setT(int T_in)
{
    if (T_in <= 2)
    {
        throw_named("Invalid number of timesteps: " << T_in);
    }
    T = T_in;
    reinitializeVariables();
}

void TimeIndexedProblem::setTau(double tau_in)
{
    if (tau_in <= 0.) throw_pretty("tau is expected to be greater than 0. (tau=" << tau_in << ")");
    tau = tau_in;
    ct = 1.0 / tau / T;
}

void TimeIndexedProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Cost.updateS();
    Inequality.updateS();
    Equality.updateS();
}

void TimeIndexedProblem::setInitialTrajectory(
    const std::vector<Eigen::VectorXd> q_init_in)
{
    if (q_init_in.size() != T)
        throw_pretty("Expected initial trajectory of length "
                     << T << " but got " << q_init_in.size());
    if (q_init_in[0].rows() != N)
        throw_pretty("Expected states to have " << N << " rows but got "
                                                << q_init_in[0].rows());

    InitialTrajectory = q_init_in;
    setStartState(q_init_in[0]);
}

std::vector<Eigen::VectorXd> TimeIndexedProblem::getInitialTrajectory()
{
    return InitialTrajectory;
}

double TimeIndexedProblem::getDuration()
{
    return tau * (double)T;
}

void TimeIndexedProblem::Update(Eigen::VectorXdRefConst x, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }

    scene_->Update(x, static_cast<double>(t) * tau);
    Phi[t].setZero(PhiN);
    J[t].setZero();
    for (int i = 0; i < NumTasks; i++)
    {
        // Only update TaskMap if Rho is not 0
        if (Tasks[i]->isUsed)
            Tasks[i]->update(x, Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length), J[t].middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
    }
    Cost.update(Phi[t], J[t], t);
    Inequality.update(Phi[t], J[t], t);
    Equality.update(Phi[t], J[t], t);
    numberOfProblemUpdates++;
}

double TimeIndexedProblem::getScalarTaskCost(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return ct * Cost.ydiff[t].transpose() * Cost.S[t] * Cost.ydiff[t];
}

Eigen::VectorXd TimeIndexedProblem::getScalarTaskJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Cost.J[t].transpose() * Cost.S[t] * Cost.ydiff[t] * 2.0 * ct;
}

double TimeIndexedProblem::getScalarTransitionCost(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return ct * xdiff[t].transpose() * W * xdiff[t];
}

Eigen::VectorXd TimeIndexedProblem::getScalarTransitionJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return 2.0 * ct * W * xdiff[t];
}

void TimeIndexedProblem::setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            Cost.y[t].data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRho(const std::string& task_name, const double rho, int t)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            Cost.Rho[t](Cost.Indexing[i].Id) = rho;
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoal(const std::string& task_name, int t)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.y[t].data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRho(const std::string& task_name, int t)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.Rho[t](Cost.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            Equality.y[t].data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRhoEQ(const std::string& task_name, const double rho, int t)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            Equality.Rho[t](Equality.Indexing[i].Id) = rho;
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoalEQ(const std::string& task_name, int t)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.y[t].data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRhoEQ(const std::string& task_name, int t)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.Rho[t](Equality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            Inequality.y[t].data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRhoNEQ(const std::string& task_name, const double rho, int t)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            Inequality.Rho[t](Inequality.Indexing[i].Id) = rho;
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoalNEQ(const std::string& task_name, int t)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.y[t].data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRhoNEQ(const std::string& task_name, int t)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.Rho[t](Inequality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}
}
