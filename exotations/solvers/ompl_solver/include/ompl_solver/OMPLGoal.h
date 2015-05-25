/*
 * OMPLGoal.h
 *
 *  Created on: 14 Jul 2014
 *      Author: s0972326
 */

#ifndef OMPLGOAL_H_
#define OMPLGOAL_H_

#include <ompl/base/Goal.h>
#include "ompl_solver/OMPLProblem.h"
#include "ompl_solver/OMPLStateSpace.h"

namespace exotica
{

    class OMPLGoal : public ompl::base::Goal
	{
		public:
			OMPLGoal (const ompl::base::SpaceInformationPtr &si, OMPLProblem_ptr prob);
            virtual	~OMPLGoal ();

			 virtual bool isSatisfied(const ompl::base::State *st) const;
			 virtual bool isSatisfied(const ompl::base::State *st, double *distance) const;

			 const OMPLProblem_ptr getProblem();
		private:
			 OMPLProblem_ptr prob_;
			 boost::shared_ptr<OMPLStateSpace> state_space_;
	};

} /* namespace exotica */

#endif /* OMPLGOAL_H_ */
