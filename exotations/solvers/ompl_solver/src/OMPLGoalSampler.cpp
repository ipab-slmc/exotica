/*
 * OMPLGoalSampler.cpp
 *
 *  Created on: 5 Aug 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLGoalSampler.h"

namespace exotica
{

	OMPLGoalSampler::OMPLGoalSampler (const ompl::base::SpaceInformationPtr &si, OMPLProblem_ptr prob, boost::shared_ptr<OMPLsolver> sol) :
		ompl::base::GoalLazySamples(si, boost::bind(&OMPLGoalSampler::sampleGoal, this, _1, _2), false),
	  prob_(prob),
	  sol_(sol)
	{

	}

	OMPLGoalSampler::~OMPLGoalSampler ()
	{

	}

	bool OMPLGoalSampler::sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
	{
		// terminate after too many attempts
		//if (attempts_so_far >= max_attempts)
		//	return false;

		// terminate after a maximum number of samples
		//if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
		//	return false;

		int max_attempts = sol_->getGoalMaxAttempts();

		// terminate the sampling thread when a solution has been found
		if (sol_->getFinishedSolving())
			return false;

		for (int a = gls->samplingAttemptsCount() ; a < max_attempts && gls->isSampling() ; ++a)
		{
			default_sampler_->sampleUniform(newGoal);
			if (static_cast<const OMPLStateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(newGoal))
			{
				return true;
			}
		}
		return false;
	}

} /* namespace exotica */
