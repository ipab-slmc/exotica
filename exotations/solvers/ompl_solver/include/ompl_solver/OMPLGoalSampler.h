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

#ifndef OMPLGOALSAMPLER_H_
#define OMPLGOALSAMPLER_H_

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "ompl_solver/OMPLsolver.h"
#include "ompl_solver/OMPLProblem.h"
#include "ompl_solver/OMPLGoal.h"
#include "generic/Identity.h"

namespace exotica
{

  class OMPLGoal;

  class OMPLsolver;

  class OMPLGoalSampler: public ompl::base::GoalSampleableRegion,
      public OMPLGoal
  {
    public:
      OMPLGoalSampler(const ompl::base::SpaceInformationPtr &si,
          OMPLProblem_ptr prob, OMPLProblem_ptr samplingBias);
      virtual
      ~OMPLGoalSampler();
      virtual void sampleGoal(ompl::base::State *st) const;
      virtual unsigned int maxSampleCount() const;
      virtual double distanceGoal(const ompl::base::State *st) const;
    private:

      OMPLProblem_ptr prob_;
      boost::shared_ptr<OMPLsolver> sol_;
      ompl::base::StateSamplerPtr default_sampler_;
      bool hasIdentityTask;
      boost::shared_ptr<Identity> taskI;
      OMPLProblem_ptr goalBias_;

  };
  typedef boost::shared_ptr<exotica::OMPLGoalSampler> OMPLGoalSampler_ptr;
} /* namespace exotica */

#endif /* OMPLGOALSAMPLER_H_ */
