/*
 * OMPLStateValidityChecker.cpp
 *
 *  Created on: 9 Jul 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLStateValidityChecker.h"

namespace exotica
{

	OMPLStateValidityChecker::OMPLStateValidityChecker(exotica::OMPLsolver* sol) :
					sol_(sol),
					ompl::base::StateValidityChecker(sol->getOMPLSimpleSetup()->getSpaceInformation()),
					prob_(sol->getProblem())
	{
	}

	OMPLStateValidityChecker::~OMPLStateValidityChecker()
	{
	}

	bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const
	{
		double tmp;
		return isValid(state, tmp);
	}

	bool OMPLStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
	{
		Eigen::VectorXd q(prob_->getSpaceDim());
		sol_->getOMPLStateSpace()->copyFromOMPLState(state, q);
		{
			boost::mutex::scoped_lock lock(prob_->getLock());
			prob_->update(q, 0);

// TODO: Implement this
// Constraints
// State rejection (e.g. collisions)
			for (auto & it : prob_->scenes_)
			{
				if (!it.second->getCollisionScene()->isStateValid())
				{
					dist = -1;
					return false;
				}
			}

			double err;
			bool terminate;
			for (TaskTerminationCriterion_ptr goal : prob_->getGoals())
			{
				if (goal->type().compare("exotica::CoM") == 0)
				{
					goal->terminate(terminate, err);
					if (!terminate)
					{
						dist = -1;
						return false;
					}
				}
			}

		}

		return true;
	}

	double OMPLStateValidityChecker::clearance(const ompl::base::State *state) const
	{
		double tmp;
		isValid(state, tmp);
		return tmp;
	}
} /* namespace exotica */
