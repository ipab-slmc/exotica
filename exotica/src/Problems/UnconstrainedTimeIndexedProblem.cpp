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

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

namespace exotica
{

  void UnconstrainedTimeIndexedProblem::update(Eigen::VectorXdRefConst x, const int t)
  {
    // Update the KinematicScene(s)...
//    for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
//    {
//      it->second->update(x);
//    }
    scene_->update(x);
    // Update task maps if the task definition precision (rho) is non-zero

    for (auto& it : task_defs_)
    {
      boost::shared_ptr<TaskSqrError> task = boost::static_pointer_cast<
          TaskSqrError>(it.second);
      if (task->getRho(t) > 0)
      {
        //task->getTaskMap()->update(x);
      }
    }
  }

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
      W = Eigen::MatrixXd::Identity(init.W.rows(), init.W.rows());
      W.diagonal() = init.W;

      for (auto& task : task_defs_)
      {
        if (task.second->type()!="exotica::TaskSqrError")
          throw_named("Task variable '" + task.first + "' is not an squared error! ("+task.second->type()+")");
      }
      // Set number of time steps
      setTime(T);
  }

  int UnconstrainedTimeIndexedProblem::getT()
  {
    return T;
  }

  void UnconstrainedTimeIndexedProblem::setTime(int T_)
  {
    if (T_ <= 0)
    {
      throw_named("Invalid number of timesteps: "<<T);
    }
    tau = (double) T * tau / (double) T_;
    T = T_;
    // Set number of time steps
    for (auto& it : task_defs_)
    {
      it.second->setTimeSteps(T + 2);
    }
  }

  void UnconstrainedTimeIndexedProblem::getT(int& T_)
  {
    T_ = T;
  }

  double UnconstrainedTimeIndexedProblem::getTau()
  {
    return tau;
  }

  void UnconstrainedTimeIndexedProblem::getTau(double& tau_)
  {
    tau_ = tau;
  }

  double UnconstrainedTimeIndexedProblem::getDuration()
  {
    return tau * (double) T;
  }

  Eigen::MatrixXd UnconstrainedTimeIndexedProblem::getW()
  {
    return W;
  }

  double UnconstrainedTimeIndexedProblem::getQrate()
  {
    return Q_rate;
  }

  double UnconstrainedTimeIndexedProblem::getWrate()
  {
    return W_rate;
  }

  double UnconstrainedTimeIndexedProblem::getHrate()
  {
    return H_rate;
  }

} /* namespace exotica */
