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

    void UnconstrainedEndPoseProblem::Instantiate(UnconstrainedEndPoseProblemInitializer& init)
    {
        NumTasks = Tasks.size();
        PhiN = 0;
        JN = 0;
        for(int i=0;i<NumTasks;i++)
        {
            appendVector(y.map, Tasks[i]->getLieGroupIndices());
            PhiN += Tasks[i]->Length;
            JN += Tasks[i]->LengthJ;
        }

        N = scene_->getNumJoints();

        Rho = Eigen::VectorXd::Ones(NumTasks);
        y.setZero(PhiN);
        W = Eigen::MatrixXd::Identity(N, N);
        if(init.W.rows()>0)
        {
            if(init.W.rows()==N)
            {
                W.diagonal() = init.W;
            }
            else
            {
                throw_named("W dimension mismatch! Expected "<<N<<", got "<<init.W.rows());
            }
        }
        Phi = y;
        J = Eigen::MatrixXd(JN, N);

        qNominal = init.NominalState;

        if(init.Rho.rows()>0 && init.Rho.rows()!=NumTasks) throw_named("Invalide size of Rho (" << init.Rho.rows() << ") expected: "<< NumTasks);
        if(init.Goal.rows()>0 && init.Goal.rows()!=PhiN) throw_named("Invalide size of Rho (" << init.Goal.rows() << ") expected: "<< PhiN);

        if(init.Rho.rows()==NumTasks) Rho = init.Rho;
        if(init.Goal.rows()==PhiN) y.data = init.Goal;
    }

    void UnconstrainedEndPoseProblem::Update(Eigen::VectorXdRefConst x)
    {
        scene_->Update(x);
        for(int i=0;i<NumTasks;i++)
        {
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length), J.middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
        }
    }

    void UnconstrainedEndPoseProblem::setGoal(const std::string & task_name, Eigen::VectorXdRefConst goal)
    {   
        try
        {
            TaskMap_ptr task = TaskMaps.at(task_name);
            if(goal.rows()!=task->Length) throw_named("Invalid goal dimension "<<goal.rows()<<" expected "<<task->Length);
            y.data.segment(task->Start, task->Length) = goal;
        }
        catch(std::out_of_range& e)
        {
            throw_pretty("Cannot set Goal. Task map '"<<task_name<<"' Does not exist.");
        }
    }

    void UnconstrainedEndPoseProblem::setRho(const std::string & task_name, const double rho)
    {
        try
        {
            TaskMap_ptr task = TaskMaps.at(task_name);
            Rho(task->Id) = rho;
        }
        catch(std::out_of_range& e)
        {
            throw_pretty("Cannot set Rho. Task map '"<<task_name<<"' Does not exist.");
        }
    }

    Eigen::VectorXd UnconstrainedEndPoseProblem::getGoal(const std::string & task_name)
    {
        try
        {
            TaskMap_ptr task = TaskMaps.at(task_name);
            return y.data.segment(task->Start, task->Length);
        }
        catch(std::out_of_range& e)
        {
            throw_pretty("Cannot get Goal. Task map '"<<task_name<<"' Does not exist.");
        }
    }

    double UnconstrainedEndPoseProblem::getRho(const std::string & task_name)
    {
        try
        {
            TaskMap_ptr task = TaskMaps.at(task_name);
            return Rho(task->Id);
        }
        catch(std::out_of_range& e)
        {
            throw_pretty("Cannot get Rho. Task map '"<<task_name<<"' Does not exist.");
        }
    }
}

