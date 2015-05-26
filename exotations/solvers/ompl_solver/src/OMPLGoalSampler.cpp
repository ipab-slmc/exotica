/*
 * OMPLGoalSampler.cpp
 *
 *  Created on: 5 Aug 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLGoalSampler.h"

namespace exotica
{

	double OMPLGoalSampler::distanceGoal(const ompl::base::State *st) const
	{
		HIGHLIGHT_NAMED("OMPL", "Distance query");
		return 0.0;
	}

	OMPLGoalSampler::OMPLGoalSampler(const ompl::base::SpaceInformationPtr &si,
			OMPLProblem_ptr prob) :
			ompl::base::GoalSampleableRegion(si), OMPLGoal(si, prob), prob_(prob)
	{
		OMPLGoal::type_ = ompl::base::GOAL_SAMPLEABLE_REGION;
		bool hasIdentityTask = false;
		for (auto& task : prob->getTaskMaps())
		{
			if (task.second->type().compare("exotica::Identity") == 0)
			{
				taskI_ = boost::static_pointer_cast<Identity>(task.second);
				hasIdentityTask = true;
				break;
			}
		}
		if (hasIdentityTask && taskI_->useRef)
		{
			goalState_ = si->allocState();
			ompl::base::RealVectorStateSpace::StateType* gstate = goalState_->as<
					ompl::base::RealVectorStateSpace::StateType>();
			int n =
					(OMPLGoal::si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>())->getDimension();
			for (int i = 0; i < n; i++)
			{
				(*gstate)[i] = taskI_->jointRef(i);
			}
		}
		else
		{
			HIGHLIGHT_NAMED("OMPL", "No configuration space goal was defined!");
			goalState_ = NULL;
		}
	}

	OMPLGoalSampler::~OMPLGoalSampler()
	{

	}

	EReturn OMPLGoalSampler::resetGoalRef()
	{
		return FAILURE;
	}
	void OMPLGoalSampler::sampleGoal(ompl::base::State *st) const
	{

		if (st)
		{
			ompl::base::RealVectorStateSpace::StateType* state = st->as<
					ompl::base::RealVectorStateSpace::StateType>();
			ompl::base::RealVectorStateSpace::StateType* state1 = goalState_->as<
					ompl::base::RealVectorStateSpace::StateType>();
			int n =
					(OMPLGoal::si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>())->getDimension();
			for (int i = 0; i < n; i++)
			{
				(*state)[i] = (*state1)[i];
			}
//			HIGHLIGHT_NAMED("OMPL", "biased towards the goal - copy");
		}
		else
		{
			st = goalState_;
			HIGHLIGHT_NAMED("OMPL", "biased towards the goal - replace");
		}
		//HIGHLIGHT_NAMED("OMPL","biased towards the goal");
	}

	unsigned int OMPLGoalSampler::maxSampleCount() const
	{
		if (goalState_)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

} /* namespace exotica */
