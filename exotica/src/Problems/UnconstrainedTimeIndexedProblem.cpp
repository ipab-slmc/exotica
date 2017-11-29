/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
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

#include <exotica/Problems/UnconstrainedTimeIndexedProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("UnconstrainedTimeIndexedProblem", exotica::UnconstrainedTimeIndexedProblem)

namespace exotica
{
UnconstrainedTimeIndexedProblem::UnconstrainedTimeIndexedProblem()
    : T(0), tau(0), Q_rate(0), W_rate(0), H_rate(0)
{
    Flags = KIN_FK | KIN_J;
}

UnconstrainedTimeIndexedProblem::~UnconstrainedTimeIndexedProblem()
{
}

void UnconstrainedTimeIndexedProblem::Instantiate(UnconstrainedTimeIndexedProblemInitializer& init)
{
    T = init.T;
    if (T <= 2)
    {
        throw_named("Invalid number of timesteps: " << T);
    }
    tau = init.Tau;
    Q_rate = init.Qrate;
    H_rate = init.Hrate;
    W_rate = init.Wrate;
    ct = 1.0 / tau / T;

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
    if (init.W.rows() > 0)
    {
        if (init.W.rows() == N)
        {
            W.diagonal() = init.W * W_rate;
        }
        else
        {
            throw_named("W dimension mismatch! Expected " << N << ", got " << init.W.rows());
        }
    }
    H = Eigen::MatrixXd::Identity(N, N) * Q_rate;
    Q = Eigen::MatrixXd::Identity(N, N) * H_rate;

    if (init.Rho.rows() == 0)
    {
        Rho.assign(T, Eigen::VectorXd::Ones(NumTasks));
    }
    else if (init.Rho.rows() == NumTasks)
    {
        Rho.assign(T, init.Rho);
    }
    else if (init.Rho.rows() == NumTasks * T)
    {
        Rho.resize(T);
        for (int i = 0; i < T; i++)
        {
            Rho[i] = init.Rho.segment(i * NumTasks, NumTasks);
        }
    }
    else
    {
        throw_named("Invalid task weights rho! " << init.Rho.rows());
    }
    yref.setZero(PhiN);
    y.assign(T, yref);
    Phi = y;
    ydiff.assign(T, Eigen::VectorXd::Zero(JN));
    J.assign(T, Eigen::MatrixXd(JN, N));
    S.assign(T, Eigen::MatrixXd::Identity(JN, JN));
    x.assign(T, Eigen::VectorXd::Zero(JN));
    xdiff.assign(T, Eigen::VectorXd::Zero(JN));

    // Set initial trajectory
    InitialTrajectory.resize(T, getStartState());
}

void UnconstrainedTimeIndexedProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int t = 0; t < T; t++)
    {
        for (TaskMap_ptr task : Tasks)
        {
            for (int i = 0; i < task->Length; i++)
            {
                S[t](i + task->Start, i + task->Start) = Rho[t](task->Id);
            }
        }
    }
}

void UnconstrainedTimeIndexedProblem::setInitialTrajectory(
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

std::vector<Eigen::VectorXd> UnconstrainedTimeIndexedProblem::getInitialTrajectory()
{
    return InitialTrajectory;
}

double UnconstrainedTimeIndexedProblem::getDuration()
{
    return tau * (double)T;
}

void UnconstrainedTimeIndexedProblem::Update(Eigen::VectorXdRefConst x_in, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }

    x[t] = x_in;
    scene_->Update(x_in, static_cast<double>(t) * tau);
    Phi[t].setZero(PhiN);
    J[t].setZero();
    for (int i = 0; i < NumTasks; i++)
    {
        // Only update TaskMap if Rho is not 0
        if (Rho[t](i) != 0)
            Tasks[i]->update(x_in, Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length), J[t].middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
    }
    ydiff[t] = Phi[t] - y[t];

    // NB: The transition cost for the 0-th time step is set to 0 in the initialiser.
    if (t > 0) xdiff[t] = x[t] - x[t - 1];

    numberOfProblemUpdates++;
}

double UnconstrainedTimeIndexedProblem::getScalarTaskCost(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return ct * ydiff[t].transpose() * S[t] * ydiff[t];
}

Eigen::VectorXd UnconstrainedTimeIndexedProblem::getScalarTaskJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return J[t].transpose() * S[t] * ydiff[t] * 2.0 * ct;
}

double UnconstrainedTimeIndexedProblem::getScalarTransitionCost(int t)
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

Eigen::VectorXd UnconstrainedTimeIndexedProblem::getScalarTransitionJacobian(int t)
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

void UnconstrainedTimeIndexedProblem::setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    try
    {
        if (t >= T || t < -1)
        {
            throw_pretty("Requested t="
                         << t
                         << " out of range, needs to be 0 =< t < " << T);
        }
        else if (t == -1)
        {
            t = T - 1;
        }

        TaskMap_ptr task = TaskMaps.at(task_name);
        y[t].data.segment(task->Start, task->Length) = goal;
    }
    catch (std::out_of_range& e)
    {
        throw_pretty("Cannot set Goal. Task map '" << task_name << "' Does not exist.");
    }
}

void UnconstrainedTimeIndexedProblem::setRho(const std::string& task_name, const double rho, int t)
{
    try
    {
        if (t >= T || t < -1)
        {
            throw_pretty("Requested t="
                         << t
                         << " out of range, needs to be 0 =< t < " << T);
        }
        else if (t == -1)
        {
            t = T - 1;
        }

        TaskMap_ptr task = TaskMaps.at(task_name);
        Rho[t](task->Id) = rho;
    }
    catch (std::out_of_range& e)
    {
        throw_pretty("Cannot set Rho. Task map '" << task_name << "' Does not exist.");
    }
}

Eigen::VectorXd UnconstrainedTimeIndexedProblem::getGoal(const std::string& task_name, int t)
{
    try
    {
        if (t >= T || t < -1)
        {
            throw_pretty("Requested t="
                         << t
                         << " out of range, needs to be 0 =< t < " << T);
        }
        else if (t == -1)
        {
            t = T - 1;
        }

        TaskMap_ptr task = TaskMaps.at(task_name);
        return y[t].data.segment(task->Start, task->Length);
    }
    catch (std::out_of_range& e)
    {
        throw_pretty("Cannot get Goal. Task map '" << task_name << "' Does not exist.");
    }
}

double UnconstrainedTimeIndexedProblem::getRho(const std::string& task_name, int t)
{
    try
    {
        if (t >= T || t < -1)
        {
            throw_pretty("Requested t="
                         << t
                         << " out of range, needs to be 0 =< t < " << T);
        }
        else if (t == -1)
        {
            t = T - 1;
        }

        TaskMap_ptr task = TaskMaps.at(task_name);
        return Rho[t](task->Id);
    }
    catch (std::out_of_range& e)
    {
        throw_pretty("Cannot get Rho. Task map '" << task_name << "' Does not exist.");
    }
}
}
