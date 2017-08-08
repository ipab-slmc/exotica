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
        throw_named("Invalid number of timesteps: "<<T);
      }
      tau = init.Tau;
      Q_rate = init.Qrate;
      H_rate = init.Hrate;
      W_rate = init.Wrate;

      NumTasks = Tasks.size();
      PhiN = 0;
      JN = 0;
      TaskSpaceVector yref;
      for(int i=0;i<NumTasks;i++)
      {
          appendVector(yref.map, Tasks[i]->getLieGroupIndices());
          PhiN += Tasks[i]->Length;
          JN += Tasks[i]->LengthJ;
      }

      N = scene_->getNumJoints();

      W = Eigen::MatrixXd::Identity(N, N)*W_rate;
      if(init.W.rows()>0)
      {
          if(init.W.rows()==N)
          {
              W.diagonal() = init.W*W_rate;
          }
          else
          {
              throw_named("W dimension mismatch! Expected "<<N<<", got "<<init.W.rows());
          }
      }
      H = Eigen::MatrixXd::Identity(N, N)*Q_rate;
      Q = Eigen::MatrixXd::Identity(N, N)*H_rate;

      Rho.assign(T, Eigen::VectorXd::Ones(NumTasks));
      yref.setZero(PhiN);
      y.assign(T, yref);
      Phi = y;
      ydiff.assign(T, Eigen::VectorXd::Zero(JN));
      J.assign(T, Eigen::MatrixXd(JN, N));
  }

    double UnconstrainedTimeIndexedProblem::getDuration()
    {
        return tau * (double) T;
    }

    void UnconstrainedTimeIndexedProblem::Update(Eigen::VectorXdRefConst x, int t)
    {
        scene_->Update(x);
        for(int i=0;i<NumTasks;i++)
        {
            Tasks[i]->update(x, Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length), J[t].middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
        }
        ydiff[t] = y[t] - Phi[t];
    }


    void UnconstrainedTimeIndexedProblem::setGoal(const std::string & task_name, Eigen::VectorXdRefConst goal, int t)
    {
        TaskMap_ptr task = TaskMaps.at(task_name);
        y[t].data.segment(task->Start, task->Length) = goal;
    }

    void UnconstrainedTimeIndexedProblem::setRho(const std::string & task_name, const double rho, int t)
    {
        TaskMap_ptr task = TaskMaps.at(task_name);
        Rho[t](task->Id) = rho;
    }

    Eigen::VectorXd UnconstrainedTimeIndexedProblem::getGoal(const std::string & task_name, int t)
    {
        TaskMap_ptr task = TaskMaps.at(task_name);
        return y[t].data.segment(task->Start, task->Length);
    }

    double UnconstrainedTimeIndexedProblem::getRho(const std::string & task_name, int t)
    {
        TaskMap_ptr task = TaskMaps.at(task_name);
        return Rho[t](task->Id);
    }
}
