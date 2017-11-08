/*
 *  Created on: 19 Jun 2014
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

#include <exotica/Problems/SamplingProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("SamplingProblem", exotica::SamplingProblem)

namespace exotica
{

SamplingTask::SamplingTask()
{

}

void SamplingTask::initialize(const std::vector<exotica::Initializer>& inits, PlanningProblem_ptr prob, TaskSpaceVector& phi)
{
    Task::initialize(inits, prob, Phi);
    y=Phi;
    y.setZero(PhiN);
    Rho = Eigen::VectorXd::Ones(NumTasks);
    S = Eigen::MatrixXd::Identity(JN, JN);
    ydiff = Eigen::VectorXd::Zero(JN);

    for(int i=0; i<NumTasks; i++)
    {
        TaskInitializer task(inits[i]);
        if(task.Goal.rows()==0)
        {
            // Keep zero goal
        }
        else if(task.Goal.rows()==Tasks[i]->Length)
        {
            y.data.segment(Tasks[i]->Start, Tasks[i]->Length) = task.Goal;
        }
        else
        {
            throw_pretty("Invalid task goal size! Expecting "<<Tasks[i]->Length<<" got "<<task.Goal.rows());
        }
        if(task.Rho.rows()==0)
        {
            Rho(i) = 1.0;
        }
        else if(task.Rho.rows()==1)
        {
            Rho(i) = task.Rho(0);
        }
        else
        {
            throw_pretty("Invalid task Rho size! Expecting 1 got "<<task.Rho.rows());
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
            if(Rho(task.Id)!=0.0) Tasks[task.Id]->isUsed = true;
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

SamplingProblem::SamplingProblem()
{
    Flags = KIN_FK;
}

SamplingProblem::~SamplingProblem()
{
    // TODO Auto-generated destructor stub
}

std::vector<double>& SamplingProblem::getBounds()
{
    return bounds_;
}

void SamplingProblem::Instantiate(SamplingProblemInitializer& init)
{
    Parameters = init;

    if (init.LocalPlannerConfig != "")
    {
        local_planner_config_ = init.LocalPlannerConfig;
    }

    goal_ = init.Goal;

    if (scene_->getBaseType() != exotica::BASE_TYPE::FIXED)
        compound_ = true;
    else
        compound_ = false;
    std::vector<std::string> jnts;
    scene_->getJointNames(jnts);

    bounds_.resize(jnts.size() * 2);
    std::map<std::string, std::vector<double>> joint_limits = scene_->getSolver().getUsedJointLimits();
    for (int i = 0; i < jnts.size(); i++)
    {
        bounds_[i] = joint_limits.at(jnts[i])[0];
        bounds_[i + jnts.size()] = joint_limits.at(jnts[i])[1];
    }

    NumTasks = Tasks.size();
    PhiN = 0;
    JN = 0;
    for (int i = 0; i < NumTasks; i++)
    {
        appendVector(Phi.map, Tasks[i]->getLieGroupIndices());
        PhiN += Tasks[i]->Length;
        JN += Tasks[i]->LengthJ;
    }
    Phi.setZero(PhiN);
    TaskSpaceVector dummy;
    Constraint.initialize(init.Constraint, shared_from_this(), dummy);
    applyStartState();
}

void SamplingProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Constraint.updateS();
}

void SamplingProblem::setGoalState(Eigen::VectorXdRefConst qT)
{
    if (qT.rows() != N)
        throw_pretty("Dimensionality of goal state wrong: Got " << qT.rows() << ", expected " << N);
    goal_ = qT;
}

bool SamplingProblem::isValid(Eigen::VectorXdRefConst x)
{
    scene_->Update(x);
    for (int i = 0; i < NumTasks; i++)
    {
        if (Tasks[i]->isUsed)
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length));
    }
    Constraint.update(Phi);
    return ((Constraint.S * Constraint.ydiff).array() < 0.0).all();
}

void SamplingProblem::Update(Eigen::VectorXdRefConst x)
{
    isValid(x);
}

int SamplingProblem::getSpaceDim()
{
    return N;
}

bool SamplingProblem::isCompoundStateSpace()
{
    return compound_;
}

} /* namespace exotica */
