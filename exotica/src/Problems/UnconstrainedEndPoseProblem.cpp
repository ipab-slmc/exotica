/*
 *  Created on: 15 Jul 2014
 *      Author: Yiming Yang
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

#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <exotica/Setup.h>
#include <exotica/TaskInitializer.h>

REGISTER_PROBLEM_TYPE("UnconstrainedEndPoseProblem", exotica::UnconstrainedEndPoseProblem)

namespace exotica
{

UnconstrainedEndPoseProblem::UnconstrainedEndPoseProblem()
{
    Flags = KIN_FK | KIN_J;
}

UnconstrainedEndPoseProblem::~UnconstrainedEndPoseProblem()
{
}

void UnconstrainedEndPoseProblem::initTaskTerms(const std::vector<exotica::Initializer>& inits)
{

}

void UnconstrainedEndPoseProblem::Instantiate(UnconstrainedEndPoseProblemInitializer& init)
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
    J = Eigen::MatrixXd(JN, N);

    if (init.NominalState.rows() > 0 && init.NominalState.rows() != N) throw_named("Invalid size of NominalState (" << init.NominalState.rows() << "), expected: " << N);
    if (init.NominalState.rows() == N) qNominal = init.NominalState;
    TaskSpaceVector dummy;
    Cost.initialize(init.Cost, shared_from_this(), dummy);
    applyStartState();
}

void UnconstrainedEndPoseProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Cost.updateS();
}

double UnconstrainedEndPoseProblem::getScalarCost()
{
    return Cost.ydiff.transpose()*Cost.S*Cost.ydiff;
}

Eigen::VectorXd UnconstrainedEndPoseProblem::getScalarJacobian()
{
    return Cost.J.transpose()*Cost.S*Cost.ydiff*2.0;
}

void UnconstrainedEndPoseProblem::Update(Eigen::VectorXdRefConst x)
{
    scene_->Update(x);
    Phi.setZero(PhiN);
    J.setZero();
    for (int i = 0; i < Tasks.size(); i++)
    {
        if (Tasks[i]->isUsed)
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length), J.middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
    }
    Cost.update(Phi, J);
}

void UnconstrainedEndPoseProblem::setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i=0; i<Cost.Indexing.size(); i++)
    {
        if(Cost.Tasks[i]->getObjectName()==task_name)
        {
            Cost.y.data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void UnconstrainedEndPoseProblem::setRho(const std::string& task_name, const double rho)
{
    for (int i=0; i<Cost.Indexing.size(); i++)
    {
        if(Cost.Tasks[i]->getObjectName()==task_name)
        {
            Cost.Rho(Cost.Indexing[i].Id) = rho;
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd UnconstrainedEndPoseProblem::getGoal(const std::string& task_name)
{
    for (int i=0; i<Cost.Indexing.size(); i++)
    {
        if(Cost.Tasks[i]->getObjectName()==task_name)
        {
            return Cost.y.data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double UnconstrainedEndPoseProblem::getRho(const std::string& task_name)
{    
    for (int i=0; i<Cost.Indexing.size(); i++)
    {
        if(Cost.Tasks[i]->getObjectName()==task_name)
        {
            return Cost.Rho(Cost.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd UnconstrainedEndPoseProblem::getNominalPose()
{
    return qNominal;
}

void UnconstrainedEndPoseProblem::setNominalPose(Eigen::VectorXdRefConst qNominal_in)
{
    if (qNominal_in.rows() == N)
        qNominal = qNominal_in;
    else
        throw_pretty("Cannot set qNominal - wrong number of rows (expected "
                     << N << ", received " << qNominal_in.rows() << ").");
}
}
