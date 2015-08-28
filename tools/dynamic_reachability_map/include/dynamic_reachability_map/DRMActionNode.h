/*
 * DRMActionNode.h
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_

#include "dynamic_reachability_map/dynamic_reachability_map.h"
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayRobotState.h>
namespace dynamic_reachability_map {
class DRMActionNode {
public:
	DRMActionNode();
	virtual ~DRMActionNode();
	bool initialise();
	bool getIKSolution(const dynamic_reachability_map::DRMGoalConstPtr &goal);
	bool updateDRM(const KDL::Frame &base_pose = KDL::Frame::Identity());
	void drmTimeCallback(const ros::TimerEvent& event);
private:
	ros::NodeHandle nh_;
	DynamicReachabilityMap drm_;
	actionlib::SimpleActionServer<dynamic_reachability_map::DRMAction> as_;
	planning_scene::PlanningScenePtr cell_ps_;

	ros::Publisher drm_pub_;
	visualization_msgs::Marker drm_mark_;
	ros::Timer drm_timer_;

	ros::Publisher cell_ps_pub_;
	ros::Publisher state_pub_;
	moveit_msgs::DisplayRobotState disp_state_;
};
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMACTIONNODE_H_ */
