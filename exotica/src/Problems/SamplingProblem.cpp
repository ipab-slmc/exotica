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
        appendVector(y.map, Tasks[i]->getLieGroupIndices());
        PhiN += Tasks[i]->Indexing[0]->Length;
        JN += Tasks[i]->Indexing[0]->LengthJ;
    }

    Rho = Eigen::VectorXd::Ones(NumTasks);
    y.setZero(PhiN);
    Phi = y;
    threshold_ = y.data;

    if (init.Rho.rows() > 0 && init.Rho.rows() != NumTasks) throw_named("Invalid size of Rho (" << init.Rho.rows() << ") expected: " << NumTasks);
    if (init.TaskGoal.rows() > 0 && init.TaskGoal.rows() != PhiN) throw_named("Invalid size of TaskGoal (" << init.TaskGoal.rows() << ") expected: " << PhiN);
    if (init.Threshold.rows() > 0 && init.Threshold.rows() != PhiN) throw_named("Invalid size of Threshold (" << init.Threshold.rows() << ") expected: " << PhiN);

    if (init.Rho.rows() == NumTasks) Rho = init.Rho;
    if (init.TaskGoal.rows() == PhiN) y.data = init.TaskGoal;
    if (init.Threshold.rows() == PhiN) threshold_ = init.Threshold;

    S = Eigen::MatrixXd::Identity(JN, JN);
    for (TaskMap_ptr task : Tasks)
    {
        for (int i = 0; i < task->Indexing[0]->Length; i++)
        {
            S(i + task->Indexing[0]->Start, i + task->Indexing[0]->Start) = Rho(task->Indexing[0]->Id);
        }
    }

    applyStartState(false);
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
        if (Rho(i) != 0)
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Indexing[0]->Start, Tasks[i]->Indexing[0]->Length));
    }
    numberOfProblemUpdates++;
    return ((S * (Phi - y) - threshold_).array() < 0.0).all();
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
