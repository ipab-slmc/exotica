/*
 * OMPLStateValidityChecker.h
 *
 *  Created on: 9 Jul 2014
 *      Author: s0972326
 */

#ifndef OMPLSTATEVALIDITYCHECKER_H_
#define OMPLSTATEVALIDITYCHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include "ompl_solver/OMPLsolver.h"
#include "ompl_solver/OMPLProblem.h"
#include <boost/thread/mutex.hpp>
namespace exotica
{

	class OMPLsolver;

	class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
	{
		public:
			OMPLStateValidityChecker (exotica::OMPLsolver* sol);
			virtual
			~OMPLStateValidityChecker ();

			virtual bool isValid(const ompl::base::State *state) const;

			virtual bool isValid(const ompl::base::State *state, double &dist) const;

			virtual double clearance(const ompl::base::State *state) const;
		protected:
			exotica::OMPLsolver* sol_;
			boost::shared_ptr<exotica::OMPLProblem> prob_;
			bool compound_;
	};

} /* namespace exotica */

#endif /* OMPLSTATEVALIDITYCHECKER_H_ */
