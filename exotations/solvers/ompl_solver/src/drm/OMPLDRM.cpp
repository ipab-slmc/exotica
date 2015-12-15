/*
 * OMPLDRM.cpp
 *
 *  Created on: 14 Dec 2015
 *      Author: yiming
 */

#include "drm/OMPLDRM.h"

namespace ompl
{
  namespace geometric
  {
    DRM::DRM(const base::SpaceInformationPtr &si)
        : traj_client_("/DRM_Trajectory", true), base::Planner(si, "DRM")
    {

    }

    DRM::~DRM()
    {

    }

    base::PlannerStatus DRM::solve(const base::PlannerTerminationCondition &ptc)
    {
      bool solved = false;
      if (!traj_client_.waitForServer(ros::Duration(2)))
      {
        ROS_ERROR("Can not connect to dynamic reachability map server");
        return base::PlannerStatus::CRASH;
      }
      unsigned int dim = si_->getStateDimension();
      dynamic_reachability_map::DRMTrajGoal goal;
      goal.ps=scene_;
      base::State *start_state = si_->allocState();
      si_->copyState(start_state, pis_.nextStart());
      {
        exotica::OMPLSE3RNCompoundStateSpace::StateType *statetype =
            static_cast<exotica::OMPLSE3RNCompoundStateSpace::StateType*>(start_state);
        goal.q0.data.resize(dim + 1);
        goal.q0.data[0] = statetype->SE3StateSpace().getX();
        goal.q0.data[1] = statetype->SE3StateSpace().getY();
        goal.q0.data[2] = statetype->SE3StateSpace().getZ();
        goal.q0.data[3] = statetype->SE3StateSpace().rotation().x;
        goal.q0.data[4] = statetype->SE3StateSpace().rotation().y;
        goal.q0.data[5] = statetype->SE3StateSpace().rotation().z;
        goal.q0.data[6] = statetype->SE3StateSpace().rotation().w;
        for (int i = 7; i < goal.q0.data.size(); i++)
          goal.q0.data[i] = statetype->RealVectorStateSpace().values[i - 7];
      }
      base::State *goal_state = si_->allocState();
      si_->copyState(goal_state, pis_.nextGoal());
      {
        exotica::OMPLSE3RNCompoundStateSpace::StateType *statetype =
            static_cast<exotica::OMPLSE3RNCompoundStateSpace::StateType*>(goal_state);
        dynamic_reachability_map::DRMTrajGoal goal;
        goal.qT.data.resize(dim + 1);
        goal.qT.data[0] = statetype->SE3StateSpace().getX();
        goal.qT.data[1] = statetype->SE3StateSpace().getY();
        goal.qT.data[2] = statetype->SE3StateSpace().getZ();
        goal.qT.data[3] = statetype->SE3StateSpace().rotation().x;
        goal.qT.data[4] = statetype->SE3StateSpace().rotation().y;
        goal.qT.data[5] = statetype->SE3StateSpace().rotation().z;
        goal.qT.data[6] = statetype->SE3StateSpace().rotation().w;
        for (int i = 7; i < goal.qT.data.size(); i++)
          goal.qT.data[i] = statetype->RealVectorStateSpace().values[i - 7];
      }

      if (traj_client_.sendGoalAndWait(goal, ros::Duration(100)).state_
          == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if (traj_client_.getResult()->succeed)
        {
          dynamic_reachability_map::DRMTrajResultConstPtr result =
              traj_client_.getResult();
          ROS_INFO_STREAM("Succeeded with length "<<result->solution.size());
          PathGeometric *path = new PathGeometric(si_);
          std::map<std::string, std::pair<int, int> > exotica_json_joints_map =
              si_->getStateSpace()->as<exotica::OMPLSE3RNCompoundStateSpace>()->exotica_json_joints_map_;
          path->append(start_state);
          for (int i = 0; i < result->solution.size(); i++)
          {
            base::State *state = si_->allocState();
            exotica::OMPLSE3RNCompoundStateSpace::StateType *statetype =
                static_cast<exotica::OMPLSE3RNCompoundStateSpace::StateType*>(state);
            statetype->SE3StateSpace().setXYZ(result->solution[i].data[0],
                result->solution[i].data[1], result->solution[i].data[2]);
            KDL::Rotation tmp_rot = KDL::Rotation::RPY(
                result->solution[i].data[3], result->solution[i].data[4],
                result->solution[i].data[5]);
            tmp_rot.GetQuaternion(statetype->SE3StateSpace().rotation().x,
                statetype->SE3StateSpace().rotation().y,
                statetype->SE3StateSpace().rotation().z,
                statetype->SE3StateSpace().rotation().w);
            for (auto &it : exotica_json_joints_map)
            {
              if (it.second.first > 5)
              {
                statetype->RealVectorStateSpace().values[it.second.first - 6] =
                    result->solution[i].data[it.second.second];
              }
            }
            path->append(state);
          }
          path->append(goal_state);
          ROS_INFO("Add solution");
          pdef_->addSolutionPath(base::PathPtr(path), false, 0, getName());
          solved = true;
        }
        else
        {
          ROS_WARN("DRM failed to find a solution");
          return base::PlannerStatus(solved, false);
        }
      }
      else
      {
        ROS_ERROR("DRM Solver Action Failed");
        return base::PlannerStatus::CRASH;
      }
      return base::PlannerStatus(solved, false);
    }

    void DRM::getPlannerData(base::PlannerData &data) const
    {
      Planner::getPlannerData(data);
    }

    void DRM::clear()
    {
      Planner::clear();
    }

    void DRM::setScene(const moveit_msgs::PlanningScene &msg)
    {
      scene_ = msg;
    }
  }
}

