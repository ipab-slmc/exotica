/*
 * OMPLDRM.h
 *
 *  Created on: 14 Dec 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_DRM_OMPLDRM_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_DRM_OMPLDRM_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "exotica/EXOTica.hpp"

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reachability_map/DRMTrajAction.h>
#include "ompl_solver/OMPLSE3RNCompoundStateSpace.h"

namespace ompl
{
  namespace geometric
  {
    class DRM: public base::Planner
    {
      public:
        DRM(const base::SpaceInformationPtr &si);

        virtual ~DRM();

        virtual base::PlannerStatus solve(
            const base::PlannerTerminationCondition &ptc);

        virtual void getPlannerData(base::PlannerData &data) const;

        virtual void clear();

        void setScene(const moveit_msgs::PlanningScene &scene);
      private:
        actionlib::SimpleActionClient<dynamic_reachability_map::DRMTrajAction> traj_client_;
        moveit_msgs::PlanningScene scene_;
    };
  }
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_DRM_OMPLDRM_H_ */
