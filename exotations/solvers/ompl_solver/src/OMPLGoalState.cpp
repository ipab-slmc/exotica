/*
 *  Created on: 17 Jun 2015
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

#include "ompl_solver/OMPLGoalState.h"

namespace exotica
{
  OMPLGoalState::OMPLGoalState(const ob::SpaceInformationPtr &si,
      OMPLProblem_ptr prob)
      : ob::GoalState(si), prob_(prob)
  {
    type_ = ob::GOAL_STATE;
    state_space_ = si->getStateSpace();
    setThreshold(std::numeric_limits<double>::epsilon());
  }

  OMPLGoalState::~OMPLGoalState()
  {

  }

  bool OMPLGoalState::isSatisfied(const ompl::base::State *st) const
  {
    return isSatisfied(st, NULL);
  }

  bool OMPLGoalState::isSatisfied(const ompl::base::State *st,
      double *distance) const
  {
    ///	First check if we have a goal configuration
    if (state_)
    {
      double dist = si_->distance(st, state_);
      if (dist > threshold_)
      {
        return false;
      }
    }
    double err = 0.0, tmp;
    bool ret = true, tmpret;
    Eigen::VectorXd q(prob_->getSpaceDim());
    if (prob_->isCompoundStateSpace())
      boost::static_pointer_cast<OMPLSE3RNStateSpace>(state_space_)->OMPLStateToEigen(
          st, q);
    else
      boost::static_pointer_cast<OMPLStateSpace>(state_space_)->copyFromOMPLState(
          st, q);
    {
      boost::mutex::scoped_lock lock(prob_->getLock());
      prob_->update(q, 0);
      for (TaskTerminationCriterion_ptr goal : prob_->getGoals())
      {
        goal->terminate(tmpret, tmp);
        if (!tmpret)
        err += tmp;
        ret = ret && tmpret;
      }
    }
    if (distance) *distance = err;
    return ret;
  }

  void OMPLGoalState::sampleGoal(ompl::base::State *st) const
  {
    if (state_) si_->copyState(st, state_);
  }

  unsigned int OMPLGoalState::maxSampleCount() const
  {
    if (state_)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  void OMPLGoalState::clearGoalState()
  {
    if (state_)
    {
      si_->freeState(state_);
      state_ = NULL;
    }
  }

  const OMPLProblem_ptr OMPLGoalState::getProblem()
  {
    return prob_;
  }
}
