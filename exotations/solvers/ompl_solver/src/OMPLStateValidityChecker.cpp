/*
 * OMPLStateValidityChecker.cpp
 *
 *  Created on: 9 Jul 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLStateValidityChecker.h"

namespace exotica
{

	OMPLStateValidityChecker::OMPLStateValidityChecker (exotica::OMPLsolver* sol) :
		sol_(sol),
		ompl::base::StateValidityChecker(sol->getOMPLSimpleSetup()->getSpaceInformation()),
    prob_(sol->getProblem())
	{
	}

	OMPLStateValidityChecker::~OMPLStateValidityChecker ()
	{
	}

	bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const
	{
		double tmp;
		isValid(state, tmp);
		return true;
	}

	bool OMPLStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
	{
		Eigen::VectorXd q(prob_->getSpaceDim());
		sol_->getOMPLStateSpace()->copyFromOMPLState(state,q);
		{
			boost::mutex::scoped_lock lock(prob_->getLock());
			prob_->update(q,0);

			// TODO: Implement this
			// Constraints
			// State rejection (e.g. collisions)

		}

		return true;
	}

	double OMPLStateValidityChecker::clearance(const ompl::base::State *state) const
	{
		double tmp;
		isValid(state,tmp);
		return tmp;
	}

} /* namespace exotica */
