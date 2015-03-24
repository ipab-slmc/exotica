/*
 * Scene.h
 *
 *  Created on: 10 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_
#define EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_

#include "exotica/Object.h"
#include "exotica/Server.h"
#include "tinyxml2/tinyxml2.h"
#include <kinematica/KinematicTree.h>

//	For collision
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>

namespace exotica
{
	typedef std::vector<collision_detection::FCLGeometryConstPtr> geos_ptr;
	typedef std::vector<boost::shared_ptr<fcl::CollisionObject> > fcls_ptr;
	//	The class of collision scene
	class CollisionScene
	{
		public:
			/*
			 * \brief	Default constructor
			 */
			CollisionScene();

			/*
			 * \brief	Destructor
			 */
			virtual ~CollisionScene();

			/*
			 * \brief	Initialisation function
			 * @param	ps		Moveit planning scene (for model building)
			 * @param	joints	Joint names
			 */
			EReturn initialise(const planning_scene::PlanningSceneConstPtr & ps,
					const std::vector<std::string> & joints);

			/*
			 * \brief	Update the robot collision properties
			 * @param	x		Configuration
			 */
			EReturn update(const Eigen::VectorXd x);

			/*
			 * \brief	Get closest distance between two objects
			 * @param	o1		Name of object 1
			 * @param	o2		Name of object 2
			 * @param	d		Closest distance (-1 if in collision)
			 * @param	p1		Closest point on o1
			 * @param	p2		Closest point on o2
			 */
			EReturn getDistance(const std::string & o1, const std::string & o2, double d);
			EReturn getDistance(const std::string & o1, const std::string & o2, double d,
					Eigen::Vector3d & p1, Eigen::Vector3d & p2);

			/*
			 * \brief	Check if the whole robot is in collision
			 * @param	self	Indicate if self collision check is required
			 */
			bool getRobotCollision(bool self);

			/*
			 * \brief	Get closest distance between robot link and anyother objects
			 * @param	link	Robot link
			 * @param	self	Indicate if self collision check is required
			 * @param	d		Closest distance
			 * @param	p1		Closest distance point on the link
			 * @param	p2		Closest distance point on the other object
			 * @param	norm	Normal vector on robot link
			 */
			EReturn getRobotDistance(const std::string & link, bool self, double & d,
					Eigen::Vector3d & p1, Eigen::Vector3d & p2, Eigen::Vector3d & norm);

			/*
			 * \brief	Get current robot state
			 * @return	Current robot state
			 */
			const robot_state::RobotState& getCurrentState();
		private:

			/*
			 * \brief	Get closest distance between two fcl objects
			 * @param	fcl1	FCL object 1
			 * @param	fcl2	FCL object 2
			 * @param	req		FCL collision request
			 * @param	res		FCL collision result
			 */
			double distance(const fcls_ptr & fcl1, const fcls_ptr & fcl2,
					const fcl::DistanceRequest & req, fcl::DistanceResult & res);
			//	FCL collision object for the robot
			std::map<std::string, fcls_ptr> fcl_robot_;

			//	FCL collision object for the world
			std::map<std::string, fcls_ptr> fcl_world_;

			//	FCL collision geometry for the robot
			std::map<std::string, geos_ptr> geo_robot_;

			//	FCL collision geometry for the world
			std::map<std::string, geos_ptr> geo_world_;

			//	Internal moveit planning scene
			planning_scene::PlanningScenePtr ps_;

			//	Joint index in robot state
			std::vector<int> joint_index_;
	};

	typedef boost::shared_ptr<CollisionScene> CollisionScene_ptr;

	//	The class of EXOTica Scene
	class Scene
	{
		public:
			/*
			 * \brief	Default constructor
			 * @param	name	The scene name
			 */
			Scene(const std::string & name);

			/*
			 * \brief	Destructor
			 */
			virtual ~Scene();

			/*
			 * \brief	Get scene name
			 * @return	Name
			 */
			std::string getName();

			/*
			 * \brief	Initialisation function
			 * @param	handle	XML handle
			 * @param	server	Server pointer
			 */
			EReturn initialisation(tinyxml2::XMLHandle & handle, const Server_ptr & server);

			/*
			 * \brief	Updator function
			 * @param	x	System state
			 */
			virtual EReturn update(const Eigen::VectorXd x, const int t = 0);

			/*
			 * \brief	Set collision scene
			 * @param	scene	Moveit planning scene
			 */
			EReturn setCollisionScene(const planning_scene::PlanningSceneConstPtr & scene);
			EReturn setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene);

			/*
			 * \brief	Append new taskmap
			 * @param	name	Taskmap name
			 * @param	eff		Endeffector names
			 * @param	offset	Endeffector offsets
			 */
			EReturn appendTaskMap(const std::string & name, const std::vector<std::string> & eff,
					const std::vector<KDL::Frame> & offset);

			/*
			 * \brief	Called after appending
			 */
			EReturn activateTaskMaps();

			/*
			 * \brief	Update task map (eff)
			 * @param	task	Task name
			 * @param	offset	Task end-effector offsets
			 */
			EReturn updateEndEffectors(const std::string & task,
					const std::vector<KDL::Frame> & offset);

			/*
			 * \breif	Get forward map
			 * @param	task	Task name
			 * @param	phi		Forward map
			 */
			EReturn getForwardMap(const std::string & task, Eigen::Ref<Eigen::VectorXd> phi);

			/*
			 * \brief	Get jacobian
			 * @param	task	Task name
			 * @param	jac		Jacobian
			 */
			EReturn getJacobian(const std::string & task, Eigen::Ref<Eigen::MatrixXd> jac);

			/*
			 * \brief	Get joint number N
			 * @return	N
			 */
			int getNumJoints();

			/*
			 * \brief	Get end-effector names for a task
			 * @param	task	Task name
			 * @param	effs	Endeffector names
			 */
			EReturn getEndEffectors(const std::string & task, std::vector<std::string> & effs);

			/*
			 * \brief	Get exotica collision scene ptr
			 * @return	CollisionScene pointer
			 */
			CollisionScene_ptr & getCollisionScene();

			/*
			 * \bref	Get map size for particular taskmap
			 * @param	task	Task name
			 * @return	Map 	Size
			 */
			int getMapSize(const std::string & task);

			/**
			 * \brief	Get centre of mass properties
			 * @param	seg		Segment names
			 * @param	mass	Mass of each link
			 * @param	cog		Centre of gravity of each link
			 * @param	tip_pose	Tip pose of each link
			 * @param	base_pose	Base pose of each link
			 */
			EReturn getCoMProperties(std::vector<std::string> & segs, Eigen::VectorXd & mass,
					std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
					std::vector<KDL::Frame> & base_pose);

			/*
			 * \brief	Get task root name
			 * @return	Root name
			 */
			std::string getRootName();

			planning_scene::PlanningScenePtr getPlanningScene();
			kinematica::KinematicTree & getSolver();
		private:

			//	ROS node handle
			ros::NodeHandle nh_;

			//	The name of the scene
			std::string name_;

			//	The kinematica tree
			kinematica::KinematicTree kinematica_;

			//	Robot model
			robot_model::RobotModelPtr model_;

			//	The controlled joint size
			int N;

			//	Initialisation flag
			bool initialised_;

			//	The big phi
			Eigen::VectorXd Phi_;

			//	The big jacobian
			Eigen::MatrixXd Jac_;

			//	Forwardmaps
			std::map<std::string, boost::shared_ptr<Eigen::Ref<Eigen::VectorXd> > > phis_;

			//	Jacobians
			std::map<std::string, boost::shared_ptr<Eigen::Ref<Eigen::MatrixXd> > > jacs_;

			//	End-effector names
			std::map<std::string, std::vector<std::string> > eff_names_;

			//	End-effector offsets
			std::map<std::string, std::vector<KDL::Frame> > eff_offsets_;

			//	End-effector index (in kinematica)
			std::map<std::string, std::vector<int> > eff_index_;

			//	Mutex locker
			boost::mutex lock_;
			CollisionScene_ptr collision_scene_;

	};
	typedef boost::shared_ptr<Scene> Scene_ptr;
	typedef std::map<std::string, Scene_ptr> Scene_map;

}	//	namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
