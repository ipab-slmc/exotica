/*
 * DRMActionNode.h
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_

#include "dynamic_reachability_map/DRM.h"
#include "dynamic_reachability_map/DRMSampleCluster.h"
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace dynamic_reachability_map
{
  class DRMActionNode
  {
    public:
      DRMActionNode();
      virtual ~DRMActionNode();
      bool initialise();
      bool getIKSolution(const dynamic_reachability_map::DRMGoalConstPtr &goal);
      bool updateDRM(const dynamic_reachability_map::DRMGoalConstPtr &goal,
          const KDL::Frame &base_pose = KDL::Frame::Identity());
      void drmTimeCallback(const ros::TimerEvent& event);
      void drmStateTimeCallback(const ros::TimerEvent& event);
      void graphTimeCallback(const ros::TimerEvent& event);
    private:
      ros::NodeHandle nh_;
      DRM_ptr drm_;
      actionlib::SimpleActionServer<dynamic_reachability_map::DRMAction> as_;
      planning_scene::PlanningScenePtr cell_ps_;

      ros::Publisher drm_pub_;
      visualization_msgs::Marker drm_mark_;
      ros::Timer drm_timer_;
      ros::Timer drm_state_timer_;
      DRMSampleCluster* drm_cluster_;
      ros::Publisher cell_ps_pub_;
      ros::Publisher state_pub_;
      moveit_msgs::DisplayRobotState disp_state_;
      moveit_msgs::DisplayTrajectory disp_traj_;
      ros::Publisher traj_pub_;

      moveit_msgs::DisplayTrajectory astar_traj_;
      ros::Publisher astar_pub_;

      visualization_msgs::MarkerArray graph_markers_;
      ros::Publisher graph_pub_;
      ros::Timer graph_timer_;

  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_ */
