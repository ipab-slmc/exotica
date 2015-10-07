/*
 * OMPLGoalState.h
 *
 *  Created on: 17 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLGOALSTATE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLGOALSTATE_H_

#include <ompl/base/goals/GoalState.h>
#include "ompl_solver/OMPLProblem.h"
#include "ompl_solver/OMPLStateSpace.h"
#include "ompl_solver/OMPLSE3RNCompoundStateSpace.h"

namespace ob = ompl::base;
namespace exotica
{
  class OMPLGoalState: public ob::GoalState
  {
    public:
      OMPLGoalState(const ob::SpaceInformationPtr &si, OMPLProblem_ptr prob);
      ~OMPLGoalState();

      ///	Functions for GOAL
      virtual bool isSatisfied(const ob::State *st) const;
      virtual bool isSatisfied(const ob::State *st, double *distance) const;

      ///	Functions for GOAL_SAMPLEABLE_REGION
      virtual void sampleGoal(ob::State *st) const;
      virtual unsigned int maxSampleCount() const;

      void clearGoalState();
      const OMPLProblem_ptr getProblem();
    private:
      OMPLProblem_ptr prob_;
      boost::shared_ptr<ob::StateSpace> state_space_;
  };
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLGOALSTATE_H_ */
