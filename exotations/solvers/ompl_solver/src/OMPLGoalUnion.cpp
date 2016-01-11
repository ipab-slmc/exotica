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

#include "ompl_solver/OMPLGoalUnion.h"
#include <ompl/base/goals/GoalLazySamples.h>

namespace exotica
{
  ompl::base::SpaceInformationPtr getGoalsSI(
      const std::vector<ompl::base::GoalPtr> &goals)
  {
    if (goals.empty()) return ompl::base::SpaceInformationPtr();
    for (std::size_t i = 0; i < goals.size(); ++i)
      if (!goals[i]->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
        throw ompl::Exception(
            "Multiplexed goals must be instances of GoalSampleableRegion");
    for (std::size_t i = 1; i < goals.size(); ++i)
      if (goals[i]->getSpaceInformation() != goals[0]->getSpaceInformation())
        throw ompl::Exception(
            "The instance of SpaceInformation must be the same among the goals to be considered");
    return goals[0]->getSpaceInformation();
  }

  OMPLGoalUnion::OMPLGoalUnion(const std::vector<ompl::base::GoalPtr> &goals)
      : ompl::base::GoalSampleableRegion(getGoalsSI(goals)), goals_(goals), gindex_(
          0)
  {
  }

  OMPLGoalUnion::~OMPLGoalUnion()
  {
  }

  void OMPLGoalUnion::startSampling()
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
      if (goals_[i]->hasType(ompl::base::GOAL_LAZY_SAMPLES))
        static_cast<ompl::base::GoalLazySamples*>(goals_[i].get())->startSampling();
  }

  void OMPLGoalUnion::stopSampling()
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
      if (goals_[i]->hasType(ompl::base::GOAL_LAZY_SAMPLES))
        static_cast<ompl::base::GoalLazySamples*>(goals_[i].get())->stopSampling();
  }

  void OMPLGoalUnion::sampleGoal(ompl::base::State *st) const
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
    {
      if (goals_[gindex_]->as<ompl::base::GoalSampleableRegion>()->maxSampleCount()
          > 0)
      {
        goals_[gindex_]->as<ompl::base::GoalSampleableRegion>()->sampleGoal(st);
        return;
      }
      gindex_ = (gindex_ + 1) % goals_.size();
    }
    throw ompl::Exception("There are no states to sample");
  }

  unsigned int OMPLGoalUnion::maxSampleCount() const
  {
    unsigned int sc = 0;
    for (std::size_t i = 0; i < goals_.size(); ++i)
      sc += goals_[i]->as<ompl::base::GoalSampleableRegion>()->maxSampleCount();
    return sc;
  }

  bool OMPLGoalUnion::canSample() const
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
      if (goals_[i]->as<ompl::base::GoalSampleableRegion>()->canSample())
        return true;
    return false;
  }

  bool OMPLGoalUnion::couldSample() const
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
      if (goals_[i]->as<ompl::base::GoalSampleableRegion>()->couldSample())
        return true;
    return false;
  }

  bool OMPLGoalUnion::isSatisfied(const ompl::base::State *st,
      double *distance) const
  {
    for (std::size_t i = 0; i < goals_.size(); ++i)
      if (goals_[i]->isSatisfied(st, distance)) return true;
    return false;
  }

  double OMPLGoalUnion::distanceGoal(const ompl::base::State *st) const
  {
    double min_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < goals_.size(); ++i)
    {
      double d = goals_[i]->as<ompl::base::GoalRegion>()->distanceGoal(st);
      if (d < min_d) min_d = d;
    }
    return min_d;
  }

  void OMPLGoalUnion::print(std::ostream &out) const
  {
    out << "MultiGoal [" << std::endl;
    for (std::size_t i = 0; i < goals_.size(); ++i)
      goals_[i]->print(out);
    out << "]" << std::endl;
  }

} /* namespace exotica */
