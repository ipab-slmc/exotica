/*
 * OMPLGoalSampler.h
 *
 *  Created on: 5 Aug 2014
 *      Author: s0972326
 */

#ifndef OMPLGOALSAMPLER_H_
#define OMPLGOALSAMPLER_H_

#include <ompl/base/goals/GoalLazySamples.h>
#include "ompl_solver/OMPLsolver.h"
#include "ompl_solver/OMPLProblem.h"

namespace exotica
{

	class OMPLsolver;

	class OMPLGoalSampler : public ompl::base::GoalLazySamples
	{
		public:
			OMPLGoalSampler (const ompl::base::SpaceInformationPtr &si, OMPLProblem_ptr prob, boost::shared_ptr<OMPLsolver> sol);
			virtual
			~OMPLGoalSampler ();
		private:
			bool sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal);

			OMPLProblem_ptr prob_;
			boost::shared_ptr<OMPLsolver> sol_;
			ompl::base::StateSamplerPtr                      default_sampler_;
	};
	typedef boost::shared_ptr<exotica::OMPLGoalSampler> OMPLGoalSampler_ptr;
} /* namespace exotica */

#endif /* OMPLGOALSAMPLER_H_ */
