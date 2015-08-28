/*
 * drm_space_sampler.cpp
 *
 *  Created on: 27 Aug 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/dynamic_reachability_map.h"
#include <ros/package.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "~");
	ros::NodeHandle nh("~");
	std::string eff, group_name;
	if (!nh.getParam("EndEffector", eff)) {
		ERROR("EndEffector link not specified [rosparam: EndEffector]");
		return 0;
	}
	if (!nh.getParam("Group", group_name)) {
		ERROR("Group name not specified [rosparam: Group]");
		return 0;
	}

	std::string bounds_string;
	if (!nh.getParam("SpaceBounds", bounds_string)) {
		ERROR("Space bounds not specified [rosparam: SpaceBounds]");
		return 0;
	}
	std::istringstream stm(bounds_string);
	std::vector<Eigen::Vector2d> bounds(3);

	try {
		stm >> bounds[0](0) >> bounds[0](1) >> bounds[1](0) >> bounds[1](1)
				>> bounds[2](0) >> bounds[2](1);
	} catch (int e) {
		ERROR("Get space bounds failed");
		return 0;
	}

	double cell_size = 0;
	if (!nh.getParam("CellSize", cell_size)) {
		ERROR("Space cell size not specified [rosparam: CellSize]");
		return 0;
	}

	int thread_cnt;
	if (!nh.getParam("ThreadsNumber", thread_cnt)) {
		WARNING("Threads number not specified [rosparam: ThreadsNumber]");
		WARNING("Sampling time can be greatly reduced by using multi threads");
		thread_cnt = 1;
	}

	int sample_cnt;
	if (!nh.getParam("SampleSize", sample_cnt)) {
		ERROR("Sample size not specified [rosparam: SampleSize]");
		return 0;
	}

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	if (!kinematic_model) {
		ERROR("Robot model not loaded");
		return 0;
	}

	dynamic_reachability_map::DynamicReachabilityMap drm;

	if (!drm.createSpace(bounds[0], bounds[1], bounds[2], cell_size)) {
		ERROR("Create dynamic reachability map space failed");
		return 0;
	}

	if (!drm.createRobotSampler(kinematic_model, eff, group_name)) {
		ERROR("Create robot sample failed");
		return 0;
	}

	dynamic_reachability_map::MultiThreadsIndexer *indexing_threads;
	indexing_threads = new dynamic_reachability_map::MultiThreadsIndexer(drm,
			thread_cnt, sample_cnt);
	indexing_threads->indexingSpace();
	indexing_threads->combineSamples(drm);
	delete indexing_threads;

	std::ofstream density_file_;
	ros::Time time = ros::Time::now();
	std::string path = ros::package::getPath("dynamic_reachability_map")
			+ "/result";
	ROS_INFO_STREAM("Saving sampling result to "<<path);
	drm.saveSpace(path, thread_cnt);
	return 0;
}

