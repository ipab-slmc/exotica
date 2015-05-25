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
        HIGHLIGHT_NAMED("OMPL","Distance query");
        return 0.0;
    }

    OMPLGoalSampler::OMPLGoalSampler (const ompl::base::SpaceInformationPtr &si, OMPLProblem_ptr prob, OMPLProblem_ptr goalBias) :
        ompl::base::GoalSampleableRegion(si), OMPLGoal(si, prob), prob_(prob)
	{
        OMPLGoal::type_ = ompl::base::GOAL_SAMPLEABLE_REGION;
        hasIdentityTask=false;
        goalBias_=goalBias;
        if(goalBias)
        {
            for(auto& task : goalBias->getTaskMaps())
            {
                if(task.second->type().compare("exotica::Identity")==0)
                {
                    taskI=boost::static_pointer_cast<Identity>(task.second);
                    hasIdentityTask=true;
                    break;
                }
            }
            if(hasIdentityTask && taskI->useRef)
            {
                HIGHLIGHT_NAMED("OMPL","Setting goal bias reference.");
            }
            else
            {
                HIGHLIGHT_NAMED("OMPL","No configuration space goal was defined!");
            }
        }
	}

	OMPLGoalSampler::~OMPLGoalSampler ()
	{

	}

    void OMPLGoalSampler::sampleGoal(ompl::base::State *st) const
    {
        if(st)
        {
            if(hasIdentityTask)
            {
                ompl::base::RealVectorStateSpace::StateType* state = st->as<ompl::base::RealVectorStateSpace::StateType>();
                int n=(OMPLGoal::si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>())->getDimension();
                for(int i=0;i<n;i++)
                {
                    (*state)[i]=taskI->jointRef(i);
                }
            }
        }
        else
        {
            INDICATE_FAILURE;
            return;
        }
    }

    unsigned int OMPLGoalSampler::maxSampleCount() const
    {
        if(hasIdentityTask)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

} /* namespace exotica */
