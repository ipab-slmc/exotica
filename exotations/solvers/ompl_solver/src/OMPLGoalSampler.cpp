/*
 *  Created on: 5 Aug 2014
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

#include "ompl_solver/OMPLGoalSampler.h"

namespace exotica
{

  double OMPLGoalSampler::distanceGoal(const ompl::base::State *st) const
  {
//		HIGHLIGHT_NAMED("OMPL", "Distance query");
//		if (st && hasIdentityTask)
//		{
//			ompl::base::State* gs = OMPLGoal::si_->allocState();
//			int n =
//					(OMPLGoal::si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>())->getDimension();
//			memcpy(gs->as<ompl::base::RealVectorStateSpace::StateType>()->values, taskI->jointRef.data(), sizeof(double)
//					* n);
//			return OMPLGoal::si_->distance(st, gs);
//		}
    return 0.0;
  }

  OMPLGoalSampler::OMPLGoalSampler(const ompl::base::SpaceInformationPtr &si,
      OMPLProblem_ptr prob, OMPLProblem_ptr goalBias)
      : OMPLGoal(si, prob), ompl::base::GoalSampleableRegion(si), prob_(prob)
  {
    OMPLGoal::type_ = ompl::base::GOAL_STATE;
    hasIdentityTask = false;
    goalBias_ = goalBias;
    if (goalBias)
    {
      for (auto& task : goalBias->getTaskMaps())
      {
        if (task.second->type().compare("exotica::Identity") == 0)
        {
          taskI = boost::static_pointer_cast<Identity>(task.second);
          hasIdentityTask = true;
          break;
        }
      }
      if (hasIdentityTask && taskI->useRef)
      {

      }
      else
      {
        HIGHLIGHT_NAMED("OMPL", "No configuration space goal was defined!");
      }
    }
  }

  OMPLGoalSampler::~OMPLGoalSampler()
  {

  }

  void OMPLGoalSampler::sampleGoal(ompl::base::State *st) const
  {
//		HIGHLIGHT_NAMED("GoalSampler", "Sample goal "<<taskI->jointRef.transpose());
    if (st && hasIdentityTask)
    {
      ompl::base::RealVectorStateSpace::StateType* state = st->as<
          ompl::base::RealVectorStateSpace::StateType>();
      int n = (OMPLGoal::si_->getStateSpace()->as<
          ompl::base::RealVectorStateSpace>())->getDimension();
      for (int i = 0; i < n; i++)
      {
        (*state)[i] = taskI->jointRef(i);
      }
    }
  }

  unsigned int OMPLGoalSampler::maxSampleCount() const
  {
    if (hasIdentityTask)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

} /* namespace exotica */
