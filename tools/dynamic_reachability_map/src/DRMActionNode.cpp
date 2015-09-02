/*
 * DRMActionNode.cpp
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMActionNode.h"
#include <ros/package.h>
void kdl2Pose(const KDL::Frame & kdl, geometry_msgs::Pose &pose) {
	pose.position.x = kdl.p.x();
	pose.position.y = kdl.p.y();
	pose.position.z = kdl.p.z();
	kdl.M.GetQuaternion(pose.orientation.x, pose.orientation.y,
			pose.orientation.z, pose.orientation.w);
}

KDL::Frame point2KDL(const geometry_msgs::Point &point) {
	return KDL::Frame(KDL::Rotation::Identity(),
			KDL::Vector(point.x, point.y, point.z));

}
KDL::Frame pose2KDL(const geometry_msgs::Pose &pose) {
	return KDL::Frame(
			KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y,
					pose.orientation.z, pose.orientation.w),
			KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

double KDLDist2D(const KDL::Vector &v1, const KDL::Vector &v2) {
	return (KDL::Vector2(v1.data[0], v1.data[1])
			- KDL::Vector2(v2.data[0], v2.data[1])).Norm();
}

namespace dynamic_reachability_map {
DRMActionNode::DRMActionNode() :
		nh_("~"), as_(nh_, "/DRM_IK",
				boost::bind(
						&dynamic_reachability_map::DRMActionNode::getIKSolution,
						this, _1), false) {
}

DRMActionNode::~DRMActionNode() {

}

bool DRMActionNode::initialise() {
	std::string model, drm_model, path, eff, group;
	if (!nh_.hasParam("RobotModel"))
		model = "robot_description";
	else
		nh_.getParam("RobotModel", model);
	if (!nh_.getParam("DRMModel", drm_model)) {
		ROS_ERROR(
				"Dynamic reachability map urdf model not defined [rosparam: DRMModel]");
		return false;
	}
	if (!nh_.getParam("DRMPath", path)) {
		ROS_ERROR(
				"Dynamic reachability map files not defined [rosparam: DRMPath]");
		return false;
	}
	if (!nh_.getParam("EndEffector", eff)) {
		ROS_ERROR("EndEffector link not defined [rosparam: EndEffector]");
		return false;
	}
	if (!nh_.getParam("Group", group)) {
		ROS_ERROR("Group name not defined [rosparam: Group]");
		return false;
	}

	robot_model_loader::RobotModelLoader robot_model_loader(model);
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	if (!drm_.createRobotSampler(kinematic_model, eff, group))
		return false;
	if (!drm_.loadSpace(path)) {
		ROS_ERROR_STREAM("Can not load dynamic reacability map from "<<path);
		return false;
	}
	drm_.computeDensity();
//	drm_timer_ = nh_.createTimer(ros::Duration(1),
//			&DRMActionNode::drmTimeCallback, this);
	robot_model_loader::RobotModelLoader cellbot_loader(drm_model);
	robot_model::RobotModelPtr cellbot_model = cellbot_loader.getModel();
	cell_ps_.reset(new planning_scene::PlanningScene(cellbot_model));
	cell_ps_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(
			"CellPlanningScene", 10);
	state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>(
			"DRMResultState", 10);
	drm_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"DynamicReachabilityMap", 10);
	drm_mark_.type = visualization_msgs::Marker::CUBE_LIST;
	drm_mark_.header.stamp = ros::Time::now();
	drm_mark_.header.frame_id = "world_frame";
	drm_mark_.scale.x = drm_mark_.scale.y = drm_mark_.scale.z = drm_.cell_size_;
	drm_mark_.action = visualization_msgs::Marker::ADD;
	as_.start();
	return true;
}

bool DRMActionNode::updateDRM(const KDL::Frame &base_pose) {
	drm_.resetDensity();
	collision_detection::CollisionRequest request;
	request.contacts = true;
	request.max_contacts = 1;
	collision_detection::CollisionResult result;

	moveit_msgs::PlanningScene msg;
	cell_ps_->getPlanningSceneMsg(msg);
	cell_ps_pub_.publish(msg);
	for (int i = 0; i < drm_.space_size_; i++) {
		KDL::Frame cell_pose = point2KDL(drm_.space_[i].centre);
		cell_pose = base_pose * cell_pose;
		cell_ps_->getCurrentStateNonConst().setVariablePosition(0,
				cell_pose.p.x());
		cell_ps_->getCurrentStateNonConst().setVariablePosition(1,
				cell_pose.p.y());
		cell_ps_->getCurrentStateNonConst().setVariablePosition(2,
				cell_pose.p.z());
		std::vector<double> quat(4);
		cell_pose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
		cell_ps_->getCurrentStateNonConst().setVariablePosition(3, quat[0]);
		cell_ps_->getCurrentStateNonConst().setVariablePosition(4, quat[1]);
		cell_ps_->getCurrentStateNonConst().setVariablePosition(5, quat[2]);
		cell_ps_->getCurrentStateNonConst().setVariablePosition(6, quat[3]);
		cell_ps_->getCurrentStateNonConst().update(true);
		cell_ps_->checkCollision(request, result);
		if (result.collision) {
			int sample_cnt = drm_.space_[i].occupiedSmaples.size();
			for (int j = 0; j < sample_cnt; j++)
				drm_.samples_[drm_.space_[i].occupiedSmaples[j]].valid = false;
		}
		result.clear();

	}

	drm_.computeDensity();

	std::vector<int> density = drm_.getDensity();
	int size = density.size();
	Eigen::VectorXd density_eigen(size);
	for (int i = 0; i < size; i++)
		density_eigen(i) = density[i];
	double max = density_eigen.maxCoeff();
	density_eigen = density_eigen / max;

	drm_mark_.colors.clear();
	drm_mark_.points.clear();
	for (int i = 0; i < size; i++) {
		if (density[i] > 0) {
			std_msgs::ColorRGBA c;
			c.a = density_eigen(i);
			c.r = 1 - density_eigen(i);
			c.g = 1;
			c.b = density_eigen(i);
			drm_mark_.colors.push_back(c);
			drm_mark_.points.push_back(drm_.space_[i].centre);
		}
	}
	if (drm_mark_.points.size() == 0)
		ERROR("Empty DRM!!!!");
	drm_pub_.publish(drm_mark_);
	return true;
}
bool DRMActionNode::getIKSolution(
		const dynamic_reachability_map::DRMGoalConstPtr &goal) {
	static bool first = true;
	if (first) {
		disp_state_.state = goal->ps.robot_state;
		first = false;
	}
	moveit_msgs::PlanningSceneWorld world = goal->ps.world;
	for (int i = 0; i < world.collision_objects.size(); i++)
		world.collision_objects[i].header.frame_id = "/world_frame";
	cell_ps_->getWorldNonConst()->clearObjects();
	cell_ps_->processPlanningSceneWorldMsg(world);
	updateDRM();
	dynamic_reachability_map::DRMResult result = drm_.getIKSolution(goal);
	as_.setSucceeded(result);
	for (int i = 0; i < result.q_out.data.size(); i++)
		disp_state_.state.joint_state.position[i] = result.q_out.data[i];
	state_pub_.publish(disp_state_);
	return result.succeed;
}

void DRMActionNode::drmTimeCallback(const ros::TimerEvent& event) {
	std::vector<int> density = drm_.getDensity();
	int size = density.size();
	Eigen::VectorXd density_eigen(size);
	for (int i = 0; i < size; i++)
		density_eigen(i) = density[i];
	double max = density_eigen.maxCoeff();
	density_eigen = density_eigen / max;

	drm_mark_.colors.clear();
	drm_mark_.points.clear();
	for (int i = 0; i < size; i++) {
		if (density[i] > 0) {
			std_msgs::ColorRGBA c;
			c.a = density_eigen(i);
			c.r = 1 - density_eigen(i);
			c.g = 1;
			c.b = density_eigen(i);
			drm_mark_.colors.push_back(c);
			drm_mark_.points.push_back(drm_.space_[i].centre);
		}
	}
	if (drm_mark_.points.size() == 0)
		ERROR("Empty DRM!!!!");
	drm_pub_.publish(drm_mark_);
}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "~");
	ros::AsyncSpinner sp(1);
	dynamic_reachability_map::DRMActionNode drm;
	if (!drm.initialise()) {
		ROS_ERROR("ERROR initialise dynamic reachability map");
		return 0;
	}
	sp.start();
	ros::waitForShutdown();
	return 0;
}
