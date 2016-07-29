/*
 *      Author: Michael Camilleri
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

#include "exotica/Definitions/TaskTerminationCriterion.h"

REGISTER_TASKDEFINITION_TYPE("TaskTerminationCriterion",
    exotica::TaskTerminationCriterion);

namespace exotica
{

  TaskTerminationCriterion::TaskTerminationCriterion()
      : threshold_(0.0)
  {
    order = 0;
    rho0_.resize(1);
    rho1_.resize(1);
    threshold0_.resize(1);
    wasFullyInitialised_ = false;
  }

  void TaskTerminationCriterion::initDerived(tinyxml2::XMLHandle & handle)
  {
    TaskSqrError::initDerived(handle);
      double thr;
      if (handle.FirstChildElement("Threshold").ToElement())
      {
        getDouble(*(handle.FirstChildElement("Threshold").ToElement()),thr);
        threshold0_(0) = thr;
      }
      else
      {
        throw_named("Threshold was not specified");
      }

    setTimeSteps(1);
  }

  void TaskTerminationCriterion::terminate(bool & end, double& err, int t)
  {
    err = ((*(task_map_->phi_.at(t))) - (*(y_star_.at(t)))).squaredNorm()
        * (*(rho_.at(t)))(0);
    end = err <= (*(threshold_.at(t)))(0);
//    	HIGHLIGHT_NAMED(object_name_,"Phi "<<task_map_->phi_.at(t)->transpose()<<" goal "<<y_star_.at(t)->transpose()<<" Err "<<err);
  }

  void TaskTerminationCriterion::registerThreshold(
      Eigen::VectorXdRef_ptr threshold, int t)
  {
    threshold_.at(t) = threshold;
  }

  void TaskTerminationCriterion::setTimeSteps(const int T)
  {
    TaskSqrError::setTimeSteps(T);
    threshold_.assign(T, Eigen::VectorXdRef_ptr(threshold0_));
  }

}
