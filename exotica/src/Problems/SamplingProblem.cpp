/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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
}

std::vector<double> SamplingProblem::getBounds()
{
    std::vector<double> bounds;
    auto jointLimits = scene_->getSolver().getJointLimits();

    bounds.resize(2 * N);
    for (unsigned int i = 0; i < N; i++)
    {
        bounds[i] = jointLimits(i, 0);
        bounds[i + N] = jointLimits(i, 1);
    }

    return bounds;
}

void SamplingProblem::Instantiate(SamplingProblemInitializer& init)
{
    Parameters = init;

    if (init.LocalPlannerConfig != "")
    {
        local_planner_config_ = init.LocalPlannerConfig;
    }

    if (init.Goal.size() == N)
    {
        goal_ = init.Goal;
    }
    else if (init.Goal.size() == 0)
    {
        goal_ = Eigen::VectorXd::Zero(N);
    }
    else
    {
        throw_named("Dimension mismatch: problem N=" << N << ", but goal state has dimension " << goal_.rows());
    }

    if (scene_->getBaseType() != exotica::BASE_TYPE::FIXED)
        compound_ = true;
    else
        compound_ = false;

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
    Inequality.initialize(init.Inequality, shared_from_this(), dummy);
    Inequality.Tolerance = init.ConstraintTolerance;
    Equality.initialize(init.Equality, shared_from_this(), dummy);
    Equality.Tolerance = init.ConstraintTolerance;
    applyStartState(false);
    preupdate();
}

void SamplingProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Inequality.updateS();
    Equality.updateS();
}

void SamplingProblem::setGoalState(Eigen::VectorXdRefConst qT)
{
    if (qT.rows() != N)
        throw_pretty("Dimensionality of goal state wrong: Got " << qT.rows() << ", expected " << N);
    goal_ = qT;
}

void SamplingProblem::setGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            if (goal.rows() != Equality.Indexing[i].Length) throw_pretty("Expected length of " << Equality.Indexing[i].Length << " and got " << goal.rows());
            Equality.y.data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void SamplingProblem::setRhoEQ(const std::string& task_name, const double rho)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            Equality.Rho(Equality.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd SamplingProblem::getGoalEQ(const std::string& task_name)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.y.data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double SamplingProblem::getRhoEQ(const std::string& task_name)
{
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.Rho(Equality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

void SamplingProblem::setGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            if (goal.rows() != Inequality.Indexing[i].Length) throw_pretty("Expected length of " << Inequality.Indexing[i].Length << " and got " << goal.rows());
            Inequality.y.data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void SamplingProblem::setRhoNEQ(const std::string& task_name, const double rho)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            Inequality.Rho(Inequality.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd SamplingProblem::getGoalNEQ(const std::string& task_name)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.y.data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double SamplingProblem::getRhoNEQ(const std::string& task_name)
{
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.Rho(Inequality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

bool SamplingProblem::isValid(Eigen::VectorXdRefConst x)
{
    scene_->Update(x);
    for (int i = 0; i < NumTasks; i++)
    {
        if (Tasks[i]->isUsed)
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length));
    }
    Inequality.update(Phi);
    Equality.update(Phi);
    numberOfProblemUpdates++;

    bool inequality_is_valid = ((Inequality.S * Inequality.ydiff).array() <= 0.0).all();
    bool equality_is_valid = ((Equality.S * Equality.ydiff).array().abs() == 0.0).all();

    return (inequality_is_valid && equality_is_valid);
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
