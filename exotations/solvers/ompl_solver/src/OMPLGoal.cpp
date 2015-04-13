/*
 * OMPLGoal.cpp
 *
 *  Created on: 14 Jul 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLGoal.h"

namespace exotica
{

	OMPLGoal::OMPLGoal (const ompl::base::SpaceInformationPtr &si, OMPLProblem_ptr prob) : ompl::base::Goal(si), prob_(prob)
	{
		state_space_=boost::static_pointer_cast<OMPLStateSpace>(si->getStateSpace());
	}

	OMPLGoal::~OMPLGoal ()
	{

	}

	bool OMPLGoal::isSatisfied(const ompl::base::State *st) const
	{
		return isSatisfied(st,NULL);
	}

  bool OMPLGoal::isSatisfied(const ompl::base::State *st, double *distance) const
  {
  	double err=0.0,tmp;
  	bool ret=true, tmpret;
  	Eigen::VectorXd q(state_space_->getDimension());
  	state_space_->copyFromOMPLState(st,q);
  	{
  		boost::mutex::scoped_lock lock(prob_->getLock());
			prob_->update(q,0);
			for(TaskTerminationCriterion_ptr goal : prob_->getGoals())
			{
				goal->terminate(tmpret,tmp);
				err+=tmp;
				ret=ret&&tmpret;
			}
  	}
  	if(distance) *distance=err;
  	return ret;
  }

} /* namespace exotica */
