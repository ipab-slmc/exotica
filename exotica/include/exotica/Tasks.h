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

namespace exotica
{
struct TaskIndexing;
class Task;
}

#ifndef EXOTICA_TASKS_H
#define EXOTICA_TASKS_H

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <exotica/PlanningProblem.h>
#include <exotica/Property.h>
#include <exotica/TaskMap.h>
#include <exotica/TaskSpaceVector.h>
#include <exotica/Tools/Conversions.h>

namespace exotica
{
struct TaskIndexing
{
    int Id;
    int Start;
    int Length;
    int StartJ;
    int LengthJ;
};

class Task
{
public:
    Task();

    virtual void initialize(const std::vector<exotica::Initializer>& inits, std::shared_ptr<PlanningProblem> prob, TaskSpaceVector& phi);

    TaskMap_map TaskMaps;
    TaskMap_vec Tasks;
    std::vector<TaskIndexing> Indexing;

    int PhiN;
    int JN;
    int NumTasks;
protected:
    std::vector<TaskInitializer> TaskInitializers;
};

class TimeIndexedTask : public Task
{
public:
    TimeIndexedTask();
    virtual void initialize(const std::vector<exotica::Initializer>& inits, std::shared_ptr<PlanningProblem> prob, TaskSpaceVector& phi);
    void updateS();
    void update(const TaskSpaceVector& Phi, Eigen::MatrixXdRefConst J, int t);
    void reinitializeVariables(int T, std::shared_ptr<PlanningProblem> prob);

    std::vector<Eigen::VectorXd> Rho;
    std::vector<TaskSpaceVector> y;
    std::vector<Eigen::VectorXd> ydiff;
    std::vector<TaskSpaceVector> Phi;
    std::vector<Eigen::MatrixXd> J;
    std::vector<Eigen::MatrixXd> S;
    int T;
};

class EndPoseTask : public Task
{
public:
    EndPoseTask();
    virtual void initialize(const std::vector<exotica::Initializer>& inits, std::shared_ptr<PlanningProblem> prob, TaskSpaceVector& phi);
    void updateS();
    void update(const TaskSpaceVector& Phi, Eigen::MatrixXdRefConst J);

    Eigen::VectorXd Rho;
    TaskSpaceVector y;
    Eigen::VectorXd ydiff;
    TaskSpaceVector Phi;
    Eigen::MatrixXd J;
    Eigen::MatrixXd S;
};

class SamplingTask : public Task
{
public:
    SamplingTask();
    virtual void initialize(const std::vector<exotica::Initializer>& inits, std::shared_ptr<PlanningProblem> prob, TaskSpaceVector& phi);
    void updateS();
    void update(const TaskSpaceVector& Phi);

    Eigen::VectorXd Rho;
    TaskSpaceVector y;
    Eigen::VectorXd ydiff;
    TaskSpaceVector Phi;
    Eigen::MatrixXd S;
};
}

#endif
