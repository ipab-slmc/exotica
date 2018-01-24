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

#include <exotica/Problems/UnconstrainedTimeIndexedProblem.h>
#include <exotica/TaskInitializer.h>
#include <exotica/Tasks.h>

namespace exotica
{
Task::Task()
{
}

void Task::initialize(const std::vector<exotica::Initializer>& inits, PlanningProblem_ptr prob, TaskSpaceVector& phi)
{
    for (const exotica::Initializer& init : inits)
    {
        TaskInitializer task(init);
        auto it = prob->getTaskMaps().find(task.Task);
        if (it == prob->getTaskMaps().end()) throw_pretty("Task map '" << task.Task << "' has not been defined!");
        TaskMaps[task.Task] = it->second;
        Tasks.push_back(it->second);
        TaskInitializers.push_back(task);
        it->second->isUsed = true;
    }
    NumTasks = Tasks.size();
    PhiN = 0;
    JN = 0;
    Indexing.resize(Tasks.size());
    for (int i = 0; i < NumTasks; i++)
    {
        Indexing[i].Id = i;
        Indexing[i].Start = PhiN;
        Indexing[i].Length = Tasks[i]->Length;
        Indexing[i].StartJ = JN;
        Indexing[i].LengthJ = Tasks[i]->LengthJ;
        appendVector(phi.map, TaskVectorEntry::reindex(Tasks[i]->getLieGroupIndices(), Tasks[i]->Start, Indexing[i].Start));
        PhiN += Tasks[i]->Length;
        JN += Tasks[i]->LengthJ;
    }
    phi.setZero(PhiN);
}

EndPoseTask::EndPoseTask()
{
}

void EndPoseTask::initialize(const std::vector<exotica::Initializer>& inits, PlanningProblem_ptr prob, TaskSpaceVector& phi)
{
    Task::initialize(inits, prob, Phi);
    y = Phi;
    y.setZero(PhiN);
    Rho = Eigen::VectorXd::Ones(NumTasks);
    J = Eigen::MatrixXd(JN, prob->N);
    S = Eigen::MatrixXd::Identity(JN, JN);
    ydiff = Eigen::VectorXd::Zero(JN);

    for (int i = 0; i < NumTasks; i++)
    {
        TaskInitializer task(inits[i]);
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == Tasks[i]->Length)
        {
            y.data.segment(Tasks[i]->Start, Tasks[i]->Length) = task.Goal;
        }
        else
        {
            throw_pretty("Invalid task goal size! Expecting " << Tasks[i]->Length << " got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            Rho(i) = 1.0;
        }
        else if (task.Rho.rows() == 1)
        {
            Rho(i) = task.Rho(0);
        }
        else
        {
            throw_pretty("Invalid task Rho size! Expecting 1 got " << task.Rho.rows());
        }
    }
}

void EndPoseTask::updateS()
{
    for (const TaskIndexing& task : Indexing)
    {
        for (int i = 0; i < task.Length; i++)
        {
            S(i + task.Start, i + task.Start) = Rho(task.Id);
            if (Rho(task.Id) != 0.0) Tasks[task.Id]->isUsed = true;
        }
    }
}

void EndPoseTask::update(const TaskSpaceVector& bigPhi, Eigen::MatrixXdRefConst bigJ)
{
    for (const TaskIndexing& task : Indexing)
    {
        Phi.data.segment(task.Start, task.Length) = bigPhi.data.segment(Tasks[task.Id]->Start, Tasks[task.Id]->Length);
        J.middleRows(task.StartJ, task.LengthJ) = bigJ.middleRows(Tasks[task.Id]->StartJ, Tasks[task.Id]->LengthJ);
    }
    ydiff = Phi - y;
}

TimeIndexedTask::TimeIndexedTask()
{
}

void TimeIndexedTask::initialize(const std::vector<exotica::Initializer>& inits, PlanningProblem_ptr prob, TaskSpaceVector& phi)
{
    Task::initialize(inits, prob, phi);
    phi.setZero(PhiN);

    T = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(prob)->getT();  // NB: Issue #227 - This cast into the UnconstrainedTimeIndexedProblem type is an assumption!
    reinitializeVariables(T, prob);
}

void TimeIndexedTask::updateS()
{
    for (int t = 0; t < T; t++)
    {
        for (const TaskIndexing& task : Indexing)
        {
            for (int i = 0; i < task.Length; i++)
            {
                S[t](i + task.Start, i + task.Start) = Rho[t](task.Id);
                if (Rho[t](task.Id) != 0.0) Tasks[task.Id]->isUsed = true;
            }
        }
    }
}

void TimeIndexedTask::update(const TaskSpaceVector& bigPhi, Eigen::MatrixXdRefConst bigJ, int t)
{
    for (const TaskIndexing& task : Indexing)
    {
        Phi[t].data.segment(task.Start, task.Length) = bigPhi.data.segment(Tasks[task.Id]->Start, Tasks[task.Id]->Length);
        J[t].middleRows(task.StartJ, task.LengthJ) = bigJ.middleRows(Tasks[task.Id]->StartJ, Tasks[task.Id]->LengthJ);
    }
    ydiff[t] = Phi[t] - y[t];
}

void TimeIndexedTask::reinitializeVariables(int T_in, PlanningProblem_ptr prob)
{
    T = T_in;
    const TaskSpaceVector& phi = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(prob)->TaskPhi;
    Phi.assign(T, phi);
    y = Phi;
    Rho.assign(T, Eigen::VectorXd::Ones(NumTasks));
    J.assign(T, Eigen::MatrixXd(JN, prob->N));
    S.assign(T, Eigen::MatrixXd::Identity(JN, JN));
    ydiff.assign(T, Eigen::VectorXd::Zero(JN));

    if (NumTasks != TaskInitializers.size()) throw_pretty("Number of tasks does not match internal number of tasks!");
    for (int i = 0; i < NumTasks; i++)
    {
        TaskInitializer& task = TaskInitializers[i];
        HIGHLIGHT("Reinitialize Task=" << task.Task << " - #" << i);
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == Tasks[i]->Length * T)
        {
            for (int t = 0; t < T; t++)
            {
                y[t].data.segment(Tasks[i]->Start, Tasks[i]->Length) = task.Goal.segment(t * Tasks[i]->Length, Tasks[i]->Length);
            }
        }
        else
        {
            throw_pretty("Invalid task goal size! Expecting " << Tasks[i]->Length * T << " got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            // Keep ones
        }
        else if (task.Rho.rows() == T)
        {
            for (int t = 0; t < T; t++)
            {
                Rho[t](i) = task.Rho(t);
            }
        }
        else
        {
            throw_pretty("Invalid task Rho size! Expecting " << T << " got " << task.Rho.rows());
        }
    }
}

SamplingTask::SamplingTask()
{
}

void SamplingTask::initialize(const std::vector<exotica::Initializer>& inits, PlanningProblem_ptr prob, TaskSpaceVector& phi)
{
    Task::initialize(inits, prob, Phi);
    y = Phi;
    y.setZero(PhiN);
    Rho = Eigen::VectorXd::Ones(NumTasks);
    S = Eigen::MatrixXd::Identity(JN, JN);
    ydiff = Eigen::VectorXd::Zero(JN);

    for (int i = 0; i < NumTasks; i++)
    {
        TaskInitializer task(inits[i]);
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == Tasks[i]->Length)
        {
            y.data.segment(Tasks[i]->Start, Tasks[i]->Length) = task.Goal;
        }
        else
        {
            throw_pretty("Invalid task goal size! Expecting " << Tasks[i]->Length << " got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            Rho(i) = 1.0;
        }
        else if (task.Rho.rows() == 1)
        {
            Rho(i) = task.Rho(0);
        }
        else
        {
            throw_pretty("Invalid task Rho size! Expecting 1 got " << task.Rho.rows());
        }
    }
}

void SamplingTask::updateS()
{
    for (const TaskIndexing& task : Indexing)
    {
        for (int i = 0; i < task.Length; i++)
        {
            S(i + task.Start, i + task.Start) = Rho(task.Id);
            if (Rho(task.Id) != 0.0) Tasks[task.Id]->isUsed = true;
        }
    }
}

void SamplingTask::update(const TaskSpaceVector& bigPhi)
{
    for (const TaskIndexing& task : Indexing)
    {
        Phi.data.segment(task.Start, task.Length) = bigPhi.data.segment(Tasks[task.Id]->Start, Tasks[task.Id]->Length);
    }
    ydiff = Phi - y;
}
}
