/*
 * EXOTicaPlannerManager.h
 *
 *  Created on: 17 May 2014
 *      Author: s0972326
 */

#ifndef EXOTICAPLANNERMANAGER_H_
#define EXOTICAPLANNERMANAGER_H_

#include <Eigen/Core>
//#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <actionlib/client/simple_action_client.h>

#include <dynamic_reconfigure/server.h>
//#include "moveit_planners_ompl/OMPLDynamicReconfigureConfig.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include "exotica_moveit/ExoticaPlanningAction.h"
#include "exotica_moveit/EXOTicaPlannerService.h"
#include "aico/AICOsolver.h"
#include <ompl_solver/OMPLsolver.h>
#include "ik_solver/ik_solver.h"
#include <exotica/Initialiser.h>
#include <ros/package.h>

namespace exotica
{

  class EXOTicaPlannerManager: public planning_interface::PlannerManager
  {
    public:
      EXOTicaPlannerManager();
      virtual
      ~EXOTicaPlannerManager();

      virtual bool initialize(const robot_model::RobotModelConstPtr& model,
          const std::string &ns);
      virtual bool canServiceRequest(
          const planning_interface::MotionPlanRequest& req) const;
      virtual std::string getDescription() const;
      virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const;

      virtual planning_interface::PlanningContextPtr getPlanningContext(
          const planning_scene::PlanningSceneConstPtr& planning_scene,
          const planning_interface::MotionPlanRequest &req,
          moveit_msgs::MoveItErrorCodes &error_code) const;

      ros::NodeHandle nh_;
      std::vector<std::string> problems_;
      std::vector<std::string> solvers_;
  };

  class EXOTicaPlanningContext: public planning_interface::PlanningContext
  {
    public:
      EXOTicaPlanningContext(const std::string &name, const std::string &group,
          const robot_model::RobotModelConstPtr& model,
          const std::string &problem_name, const std::string &solver_name);
      bool configure(const planning_scene::PlanningSceneConstPtr & scene);

      /** \brief Solve the motion planning problem and store the result in \e res. This function should not clear data structures before computing. The constructor and clear() do that. */
      virtual bool solve(planning_interface::MotionPlanResponse &res);

      /** \brief Solve the motion planning problem and store the detailed result in \e res. This function should not clear data structures before computing. The constructor and clear() do that. */
      virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

      /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if solve() is not running (returns true).*/
      virtual bool terminate();

      /** \brief Clear the data structures used by the planner */
      virtual void clear();

      void copySolution(const Eigen::Ref<const Eigen::MatrixXd> & solution,
          robot_trajectory::RobotTrajectory* traj);

      void setCompleteInitialState(
          const robot_state::RobotState &complete_initial_robot_state);

      MotionSolver_ptr sol;
      Eigen::MatrixXd solution_;
      robot_state::RobotState start_state_;
      robot_state::RobotState goal_state_;
      double tau_;
      ros::NodeHandle nh_;
      Server_ptr ser_;
      std::string problem_name_;
      std::string solver_name_;
      std::vector<std::string> used_names_;
      std::string config_file_;
      actionlib::SimpleActionClient<exotica_moveit::ExoticaPlanningAction> client_;
  };

} /* namespace exotica */

#endif /* EXOTICAPLANNERMANAGER_H_ */
