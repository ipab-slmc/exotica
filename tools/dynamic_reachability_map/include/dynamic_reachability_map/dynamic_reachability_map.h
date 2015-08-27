/*
 * dynamic_reachability_map.h
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DYNAMIC_REACHABILITY_MAP_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DYNAMIC_REACHABILITY_MAP_H_

#include <ros/ros.h>
#include <exotica/Tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <kdl/frames.hpp>
#include <boost/function/function0.hpp>
#include <boost/thread/thread.hpp>
#include "dynamic_reachability_map/DRMAction.h"
namespace dynamic_reachability_map {
struct Sample {
	Eigen::VectorXd q;
	bool valid;
	geometry_msgs::Pose tcp;
	int tcpCell;
};
struct Cell {
	int index;
	geometry_msgs::Point centre;
	Eigen::Vector3i space_index;
	std::vector<int> occupiedSmaples;
	bool isFree;
};

struct TCP {
	int index;
	std::vector<int> samples;
};

struct AStarNode {
	AStarNode() :
			g(0.0), h(0.0), parent(-1), sample_index(-1) {

	}
	AStarNode(double g_, double h_) :
			g(g_), h(h_), parent(-1), sample_index(-1) {

	}
	double f() {
		return g + h;
	}
	double g;
	double h;
	int sample_index;
	int parent;
};

class DynamicReachabilityMap {
public:
	DynamicReachabilityMap();
	virtual ~DynamicReachabilityMap();
	/*
	 * \brief	Create a indexed grid space
	 * @param	bx		Lower/upper bounds of X axis
	 * @param	by		Lower/upper bounds of Y axis
	 * @param	bz		Lower/upper bounds of Z axis
	 * @return	True if succeeded, false otherwise
	 */
	bool createSpace(const Eigen::Vector2d & bx, const Eigen::Vector2d & by,
			const Eigen::Vector2d & bz, double cellsize);

	/*
	 * \brief	Create a robot model and planning scene for sampling
	 * @param	model	Robot model
	 * @param	eff		Tip link name
	 */
	bool createRobotSampler(const robot_model::RobotModelConstPtr &model,
			const std::string & eff, const std::string & group_name);
	bool createSpacePlanningScene(moveit_msgs::PlanningSceneWorld &world,
			std::map<std::string, int> &cell_name_map);
	bool setSpacePlanningScene(const moveit_msgs::PlanningSceneWorld world);
	bool indexingSpace(int sample_cnt,
			std::map<std::string, int> cell_name_map);
	void getSpaceCentres(std::vector<geometry_msgs::Point> & centres);
	void resetDensity();
	void computeDensity();
	std::vector<int> &getDensity();
	bool update(const geometry_msgs::Point & point, int &index);
	bool updatePlanningScene(const geometry_msgs::Point & point, int &index);
	bool solve(const Eigen::VectorXd & q0, const geometry_msgs::Point & goal,
			Eigen::MatrixXd & solution, Eigen::MatrixXd & smooth_solution,
			std::vector<int> & sample_path);

	dynamic_reachability_map::DRMResult getIKSolution(
			const dynamic_reachability_map::DRMGoalConstPtr &goal);

	bool AStarPathPlanning(int start, const Eigen::VectorXd &start_q, int goal,
			std::vector<int> &path);

	double getHeuristicDist(int a, int b);
	std::map<int, double> getNeighbors(int index);
	bool checkPath(const std::vector<int> &sample_path);
	bool checkPathDetail(const std::vector<int> &sample_path,
			std::pair<int, int> &disconnect);
	bool checkPathUsePlanningScene(const Eigen::MatrixXd &solution);

	int getIndex(const geometry_msgs::Point & point);

	bool saveSpace(const std::string & path, int multiThreads = 1);
	bool loadSpace(const std::string & path);

	bool kMeansClustering();
	bool createCellURDF(const std::string & path);
	void pathSimplifier(const Eigen::MatrixXd & path, Eigen::MatrixXd &simple);
	bool isStateValid(const ompl::base::State *state);
//		private:

	std::vector<Cell> space_;
	std::vector<Sample> samples_;
	std::vector<TCP> tcps_;
	std::vector<int> density_;
	int space_size_;
	int sample_size_;
	int dimension_;
	double cell_size_;
	std::vector<Eigen::Vector2d> bounds_;
	Eigen::Vector3i cell_cnts_;
	planning_scene::PlanningScenePtr ps_;
	std::string eff_;
	const robot_state::JointModelGroup *group_;
	std::vector<std::string> joint_names_;
	std::string root_link_;
	std::vector<int> var_index_;

	std::vector<int> sample_path_;
	int thread_id_;
};

class MultiThreadsIndexer {
public:
	MultiThreadsIndexer(DynamicReachabilityMap &is, int thread_cnt,
			int sample_cnt) :
			thread_cnt_(thread_cnt), sample_size_(sample_cnt) {
		drm_.resize(thread_cnt);

		is.createSpacePlanningScene(world_, cell_name_map_);
		for (int i = 0; i < thread_cnt; i++) {
			drm_[i].createSpace(is.bounds_[0], is.bounds_[1], is.bounds_[2],
					is.cell_size_);
			drm_[i].createRobotSampler(is.ps_->getRobotModel(), is.eff_,
					is.group_->getName());
			drm_[i].thread_id_ = i;
		}
	}
	void indexingSpace() {
		ros::Time start = ros::Time::now();
		std::vector<boost::thread*> th(thread_cnt_);
		for (unsigned int i = 0; i < thread_cnt_; ++i)
			th[i] = new boost::thread(
					boost::bind(&MultiThreadsIndexer::indexing, this, i,
							sample_size_ / thread_cnt_));
		for (unsigned int i = 0; i < thread_cnt_; ++i) {
			th[i]->join();
			delete th[i];
		}
		ROS_WARN_STREAM(
				"Multi Threads Indexing finished in "<<ros::Duration(ros::Time::now()-start).toSec()<<"sec, "<<sample_size_<<" states are sampled");
	}

	bool combineSamples(DynamicReachabilityMap &is) {
		is.sample_size_ = sample_size_;
		is.samples_.reserve(sample_size_);
		int current_start = 0;
		for (int j = 0; j < is.space_size_; j++) {
			is.space_[j].occupiedSmaples.clear();
			is.tcps_[j].samples.clear();
		}
		for (int i = 0; i < thread_cnt_; i++) {
			ROS_INFO_STREAM("Inserting samples in thread "<<i);
			is.samples_.insert(is.samples_.end(), drm_[i].samples_.begin(),
					drm_[i].samples_.end());
			for (int j = 0; j < is.space_size_; j++) {
				for (int l = 0; l < drm_[i].space_[j].occupiedSmaples.size();
						l++) {
					int tmp = drm_[i].space_[j].occupiedSmaples[l]
							+ current_start;
					is.space_[j].occupiedSmaples.push_back(tmp);
				}
				for (int l = 0; l < drm_[i].tcps_[j].samples.size(); l++) {
					int tmp = drm_[i].tcps_[j].samples[l] + current_start;
					is.tcps_[j].samples.push_back(tmp);
				}
			}
			current_start += drm_[i].sample_size_;
		}

		return true;
	}
private:
	void indexing(int id, int sample_cnt) {
		srand(time(NULL));
		drm_[id].setSpacePlanningScene(world_);
		drm_[id].indexingSpace(sample_cnt, cell_name_map_);
	}
	moveit_msgs::PlanningSceneWorld world_;
	std::map<std::string, int> cell_name_map_;
	std::vector<DynamicReachabilityMap> drm_;
	int sample_size_;
	int thread_cnt_;
};

class MultiThreadsSpaceOccupationLoader {
public:
	MultiThreadsSpaceOccupationLoader(const std::string & path, int thread_cnt,
			int space_size, int sample_size) :
			path_(path), thread_cnt_(thread_cnt), space_size_(space_size), sample_size_(
					sample_size) {
		space_occup_.resize(thread_cnt);
	}
	void loadSpaceOccupation() {
		std::vector<boost::thread*> th(thread_cnt_);
		for (unsigned int i = 0; i < thread_cnt_; ++i)
			th[i] = new boost::thread(
					boost::bind(&MultiThreadsSpaceOccupationLoader::load, this,
							i, space_size_ / thread_cnt_));
		for (unsigned int i = 0; i < thread_cnt_; ++i) {
			th[i]->join();
			delete th[i];
		}
	}
	std::vector<std::vector<std::vector<int>>>space_occup_;
private:
	void load(int id, int size);
	std::string path_;
	int thread_cnt_;
	int space_size_;
	int sample_size_;
};

}
#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DYNAMIC_REACHABILITY_MAP_H_ */
