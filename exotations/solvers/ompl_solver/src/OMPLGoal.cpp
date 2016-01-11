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

#include "ompl_solver/OMPLGoal.h"

namespace exotica
{

  OMPLGoal::OMPLGoal(const ompl::base::SpaceInformationPtr &si,
      OMPLProblem_ptr prob)
      : ompl::base::Goal(si), prob_(prob)
  {
    state_space_ = boost::static_pointer_cast<OMPLStateSpace>(
        si->getStateSpace());
  }

  OMPLGoal::~OMPLGoal()
  {

  }

  bool OMPLGoal::isSatisfied(const ompl::base::State *st) const
  {
    return isSatisfied(st, NULL);
  }

  bool OMPLGoal::isSatisfied(const ompl::base::State *st,
      double *distance) const
  {
    double err = 0.0, tmp;
    bool ret = true, tmpret;
    Eigen::VectorXd q(state_space_->getDimension());
    state_space_->copyFromOMPLState(st, q);
    {
      boost::mutex::scoped_lock lock(prob_->getLock());
      prob_->update(q, 0);
      for (TaskTerminationCriterion_ptr goal : prob_->getGoals())
      {
        goal->terminate(tmpret, tmp);
        err += tmp;
        ret = ret && tmpret;
      }
    }
    if (distance) *distance = err;
    return ret;
  }

  const OMPLProblem_ptr OMPLGoal::getProblem()
  {
    return prob_;
  }

} /* namespace exotica */
