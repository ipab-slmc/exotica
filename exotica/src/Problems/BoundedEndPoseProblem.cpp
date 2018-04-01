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

#include <exotica/Problems/BoundedEndPoseProblem.h>
#include <exotica/Setup.h>
#include <exotica/TaskInitializer.h>

REGISTER_PROBLEM_TYPE("BoundedEndPoseProblem", exotica::BoundedEndPoseProblem)

namespace exotica
{
BoundedEndPoseProblem::BoundedEndPoseProblem()
{
    Flags = KIN_FK | KIN_J;
}

BoundedEndPoseProblem::~BoundedEndPoseProblem()
{
}

void BoundedEndPoseProblem::initTaskTerms(const std::vector<exotica::Initializer>& inits)
{
}

Eigen::MatrixXd& BoundedEndPoseProblem::getBounds()
{
    return bounds_;
}

void BoundedEndPoseProblem::Instantiate(BoundedEndPoseProblemInitializer& init)
{
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
    W = Eigen::MatrixXd::Identity(N, N);
    if (init.W.rows() > 0)
    {
        if (init.W.rows() == N)
        {
            W.diagonal() = init.W;
        }
        else
        {
            throw_named("W dimension mismatch! Expected " << N << ", got " << init.W.rows());
        }
    }
    if (Flags & KIN_J) J = Eigen::MatrixXd(JN, N);
    if (Flags & KIN_J_DOT) H.setConstant(JN, Eigen::MatrixXd::Zero(N, N));

    bounds_ = scene_->getSolver().getJointLimits();
    if (init.LowerBound.rows() == N)
    {
        bounds_.col(0) = init.LowerBound;
    }
    else if (init.LowerBound.rows() != 0)
    {
        throw_named("Lower bound size incorrect! Expected " << N << " got " << init.LowerBound.rows());
    }
    if (init.UpperBound.rows() == N)
    {
        bounds_.col(1) = init.UpperBound;
    }
    else if (init.UpperBound.rows() != 0)
    {
        throw_named("Lower bound size incorrect! Expected " << N << " got " << init.UpperBound.rows());
    }

    TaskSpaceVector dummy;
    Cost.initialize(init.Cost, shared_from_this(), dummy);
    applyStartState(false);
    preupdate();
}

void BoundedEndPoseProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Cost.updateS();
}

double BoundedEndPoseProblem::getScalarCost()
{
    return Cost.ydiff.transpose() * Cost.S * Cost.ydiff;
}

Eigen::VectorXd BoundedEndPoseProblem::getScalarJacobian()
{
    return Cost.J.transpose() * Cost.S * Cost.ydiff * 2.0;
}

double BoundedEndPoseProblem::getScalarTaskCost(const std::string& task_name)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.ydiff.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length).transpose() * Cost.Rho(Cost.Indexing[i].Id) * Cost.ydiff.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get scalar task cost. Task map '" << task_name << "' does not exist.");
}

void BoundedEndPoseProblem::Update(Eigen::VectorXdRefConst x)
{
    scene_->Update(x, tStart);
    Phi.setZero(PhiN);
    if (Flags & KIN_J) J.setZero();
    if (Flags & KIN_J_DOT)
        for (int i = 0; i < JN; i++) H(i).setZero();
    for (int i = 0; i < Tasks.size(); i++)
    {
        if (Tasks[i]->isUsed)
        {
            if (Flags & KIN_J_DOT)
            {
                Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length), J.middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ), H.segment(Tasks[i]->Start, Tasks[i]->Length));
            }
            else if (Flags & KIN_J)
            {
                Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length), J.middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
            }
            else
            {
                Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length));
            }
        }
    }
    if (Flags & KIN_J_DOT)
    {
        Cost.update(Phi, J, H);
    }
    else if (Flags & KIN_J)
    {
        Cost.update(Phi, J);
    }
    else
    {
        Cost.update(Phi);
    }
    numberOfProblemUpdates++;
}

void BoundedEndPoseProblem::setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            Cost.y.data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void BoundedEndPoseProblem::setRho(const std::string& task_name, const double rho)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            Cost.Rho(Cost.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd BoundedEndPoseProblem::getGoal(const std::string& task_name)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.y.data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double BoundedEndPoseProblem::getRho(const std::string& task_name)
{
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.Rho(Cost.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

bool BoundedEndPoseProblem::isValid()
{
    Eigen::VectorXd x = scene_->getSolver().getControlledState();
    for (unsigned int i = 0; i < N; i++)
    {
        if (x(i) < bounds_(i, 0) || x(i) > bounds_(i, 1)) return false;
    }
    return true;
}
}
