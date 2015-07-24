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
#include "kinematica/KinematicTree.h"

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

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
namespace exotica
{
	typedef std::vector<collision_detection::FCLGeometryConstPtr> geos_ptr;
	typedef std::vector<boost::shared_ptr<fcl::CollisionObject> > fcls_ptr;
	enum BASE_TYPE
	{
		FIXED = 0, FLOATING = 10, PLANAR = 20
	};
	///	The class of collision scene
	class CollisionScene
	{
		public:
			/**
			 * \brief	Default constructor
			 * @param	server	Pointer to exotica server
			 */
			CollisionScene(const Server_ptr & server, const std::string & scene_name);

			/**
			 * \brief	Destructor
			 */
			virtual ~CollisionScene();

			/**
			 * \brief	Initialisation function
			 * @param	psmsg	Moveit planning scene message
			 * @param	joints	Joint names
			 * @param	mode	Update mode
			 * @return Indication of success
			 */
			EReturn initialise(const moveit_msgs::PlanningSceneConstPtr & psmsg,
					const std::vector<std::string> & joints, std::string & mode, BASE_TYPE base_type);
			EReturn reinitialise();

			/**
			 * \brief	Update the robot collision properties
			 * @param	x		Configuration
			 * @return Indication of success
			 */
			EReturn update(Eigen::VectorXdRefConst x);

			/**
			 * \brief	Get closest distance between two objects
			 * @param	o1		Name of object 1
			 * @param	o2		Name of object 2
			 * @param	d		Closest distance (-1 if in collision)
			 * @return Indication of success
			 */
			EReturn getDistance(const std::string & o1, const std::string & o2, double& d,
					double safeDist);
			/**
			 * @brief Get closest distance between two objects
			 * @param	o1		Name of object 1
			 * @param	o2		Name of object 2
			 * @param	d		Closest distance (-1 if in collision)
			 * @param	p1		Closest point on o1
			 * @param	p2		Closest point on o2
			 * @return Indication of success
			 */
			EReturn getDistance(const std::string & o1, const std::string & o2, double& d,
					Eigen::Vector3d & p1, Eigen::Vector3d & p2, double safeDist);

			/**
			 * \brief	Check if the whole robot is valid (collision and feasibility)
			 * @param	self	Indicate if self collision check is required
			 * @return True, if the state is collision free.
			 */
			bool isStateValid(bool self = true);

			/**
			 * \brief	Get closest distance between robot link and anyother objects
			 * @param	link	Robot link
			 * @param	self	Indicate if self collision check is required
			 * @param	d		Closest distance
			 * @param	p1		Closest distance point on the link
			 * @param	p2		Closest distance point on the other object
			 * @param	norm	Normal vector on robot link
			 * @return Indication of success
			 */
			EReturn getRobotDistance(const std::string & link, bool self, double & d,
					Eigen::Vector3d & p1, Eigen::Vector3d & p2, Eigen::Vector3d & norm,
					Eigen::Vector3d & c1, Eigen::Vector3d & c2, double safeDist);

			/**
			 * \brief	Get current robot state
			 * @return	Current robot state
			 */
			const robot_state::RobotState& getCurrentState();

			/**
			 * \brief	Get the moveit planning scene
			 * @return	Moveit planning scene
			 */
			planning_scene::PlanningScenePtr getPlanningScene();

			inline std::map<std::string, fcls_ptr>& getFCLWorld()
			{
				return fcl_world_;
			}

			inline std::map<std::string, fcls_ptr>& getFCLRobot()
			{
				return fcl_robot_;
			}

			EReturn updateWorld(const moveit_msgs::PlanningSceneWorldConstPtr & world);
			EReturn getCollisionLinkTranslation(const std::string & name,
					Eigen::Vector3d & translation);
			EReturn getWorldObjectTranslation(const std::string & name,
					Eigen::Vector3d & translation);
			EReturn getTranslation(const std::string & name, Eigen::Vector3d & translation);
			int stateCheckCnt_;
		private:

			/**
			 * \brief	Get closest distance between two fcl objects
			 * @param	fcl1	FCL object 1
			 * @param	fcl2	FCL object 2
			 * @param	req		FCL collision request
			 * @param	res		FCL collision result
			 * @return Distance to collision
			 */
			double distance(const fcls_ptr & fcl1, const fcls_ptr & fcl2,
					const fcl::DistanceRequest & req, fcl::DistanceResult & res, double safeDist);
			///	FCL collision object for the robot
			std::map<std::string, fcls_ptr> fcl_robot_;

			///	FCL collision object for the world
			std::map<std::string, fcls_ptr> fcl_world_;

			///	FCL collision geometry for the robot
			std::map<std::string, geos_ptr> geo_robot_;

			///	FCL collision geometry for the world
			std::map<std::string, geos_ptr> geo_world_;

			///	To correct FCL transform
			std::map<std::string, std::vector<fcl::Transform3f>> trans_world_;

			///	Internal moveit planning scene
			planning_scene::PlanningScenePtr ps_;

			///	Joint index in robot state
			std::vector<int> joint_index_;

			///	Indicate if distance computation is required
			bool compute_dist;

			///	The allowed collisiom matrix
			collision_detection::AllowedCollisionMatrix acm_;

			///	Pointer to exotica server
			exotica::Server_ptr server_;
			std::string scene_name_;
			BASE_TYPE base_type_;

	};

	typedef boost::shared_ptr<CollisionScene> CollisionScene_ptr;

	///	The class of EXOTica Scene
	class Scene: public Object
	{
		public:
			/**
			 * \brief	Default constructor
			 * @param	name	The scene name
			 */
			Scene(const std::string & name);

			/**
			 * \brief	Destructor
			 */
			virtual ~Scene();

			/**
			 * \brief	Get scene name
			 * @return	Name
			 */
			std::string getName();

			/**
			 * \brief	Initialisation function
			 * @param	handle	XML handle
			 * @param	server	Server pointer
			 * @return Indication of success
			 */
			EReturn initialisation(tinyxml2::XMLHandle & handle, const Server_ptr & server);

			/**
			 * \brief	Updator function
			 * @param	x	System state
			 * @return Indication of success
			 */
			virtual EReturn update(Eigen::VectorXdRefConst x, const int t = 0);

			/**
			 * \brief	Set collision scene
			 * @param	scene	Moveit planning scene
			 * @return Indication of success
			 */
			EReturn setCollisionScene(const planning_scene::PlanningSceneConstPtr & scene);

			/**
			 * @brief Set collision scene
			 * @param scene Planning scene message
			 * @return Indication of success
			 */
			EReturn setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene);

			/**
			 * \brief	Append new taskmap
			 * @param	name	Taskmap name
			 * @param	eff		Endeffector names
			 * @param	offset	Endeffector offsets
			 * @return Indication of success
			 */
			EReturn appendTaskMap(const std::string & name, const std::vector<std::string> & eff,
					const std::vector<KDL::Frame> & offset);

			/**
			 * \brief	Called after appending
			 * @return Indication of success
			 */
			EReturn activateTaskMaps();

			/**
			 * \brief	Update task map (eff)
			 * @param	task	Task name
			 * @param	offset	Task end-effector offsets
			 * @return Indication of success
			 */
			EReturn updateEndEffectors(const std::string & task,
					const std::vector<KDL::Frame> & offset);

			/**
			 * \brief	Get forward map (values)
			 * @param	task	Task name
			 * @param	phi		Returned forward map
			 * @return Indication of success
			 */
			EReturn getForwardMap(const std::string & task, Eigen::VectorXdRef phi);

			/**
			 * @brief Get forward map reference
			 * @param task Task name
			 * @param phi Returned forward map reference
			 * @return Indication of success
			 */
			EReturn getForwardMap(const std::string & task, Eigen::VectorXdRef_ptr& phi,
					bool force = false);

			/**
			 * \brief	Get jacobian (values)
			 * @param	task	Task name
			 * @param	jac		Returned Jacobian
			 * @return Indication of success
			 */
			EReturn getJacobian(const std::string & task, Eigen::MatrixXdRef jac);

			/**
			 * @brief Get jacobian reference
			 * @param task Task name
			 * @param jac Returned Jacobian reference
			 * @return Indication of success
			 */
			EReturn getJacobian(const std::string & task, Eigen::MatrixXdRef_ptr& jac, bool force =
					false);

			/**
			 * \brief	Get joint number N
			 * @return	N
			 */
			int getNumJoints();

			/**
			 * \brief	Get end-effector names for a task
			 * @param	task	Task name
			 * @param	effs	Endeffector names
			 * @return Indication of success
			 */
			EReturn getEndEffectors(const std::string & task, std::vector<std::string> & effs);

			/**
			 * \brief	Get exotica collision scene ptr
			 * @return	CollisionScene pointer
			 */
			CollisionScene_ptr & getCollisionScene();

			/**
			 * \brief	Get map size for particular taskmap
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
			 * @return Indication of success
			 */
			EReturn getCoMProperties(std::string& task, std::vector<std::string> & segs,
					Eigen::VectorXd & mass, std::vector<KDL::Vector> & cog,
					std::vector<KDL::Frame> & tip_pose, std::vector<KDL::Frame> & base_pose);

			/**
			 * \brief	Get task root name
			 * @return	Root name
			 */
			std::string getRootName();

			/**
			 * @brief getPlanningScene Returns the MoveIt! planning scene associated with this Scene
			 * @return Planning Scene
			 */
			planning_scene::PlanningScenePtr getPlanningScene();

			/**
			 * @brief getSolver Returns the instance of Kinematica solver associated with this Scene
			 * @return Kinematic tree solver
			 */
			kinematica::KinematicTree & getSolver();

			/**
			 * @brief Returns poses ofrequested end-effectors
			 * @param names List of end-effector names
			 * @param poses Returned poses
			 * @return Indication of success
			 */
			EReturn getPoses(const std::vector<std::string> & names,
					std::vector<KDL::Frame> & poses);

			/**
			 * @brief getRobotModel Returns the robot model
			 * @return Robot model
			 */
			robot_model::RobotModelPtr getRobotModel();

			/**
			 * \brief	Get controlled joint names
			 * @param	joints	Joint names
			 */
			EReturn getJointNames(std::vector<std::string> & joints);

			/*
			 * \brief	Get planning mode
			 * @return	Planning mode
			 */
			std::string & getPlanningMode();

			/*
			 * \breif	Get robot base type
			 * @return	Base type
			 */
			BASE_TYPE getBaseType()
			{
				return base_type_;
			}

			KDL::Frame getRobotRootWorldTransform();
		private:
			///	EXOTica server
			Server_ptr server_;

			///	The name of the scene
			std::string name_;

			///	The kinematica tree
			kinematica::KinematicTree kinematica_;

			///	Robot model
			robot_model::RobotModelPtr model_;

			///	Robot base type
			BASE_TYPE base_type_;

			///	The controlled joint size
			int N;

			///	Initialisation flag
			bool initialised_;

			///	The big phi
			Eigen::VectorXd Phi_;

			///	The big jacobian
			Eigen::MatrixXd Jac_;

			///	Forwardmaps referring to subvectors of the big phi
			std::map<std::string, Eigen::VectorXdRef_ptr> phis_;

			///	Jacobians referring to submatrices of the big jacobian
			std::map<std::string, Eigen::MatrixXdRef_ptr> jacs_;

			///	End-effector names
			std::map<std::string, std::vector<std::string> > eff_names_;

			///	End-effector offsets
			std::map<std::string, std::vector<KDL::Frame> > eff_offsets_;

			///	End-effector index (in kinematica)
			std::map<std::string, std::vector<int> > eff_index_;

			///	Mutex locker
			boost::mutex lock_;

			///	The collision scene
			CollisionScene_ptr collision_scene_;

			///	Update mode (Sampling or Optimization)
			EParam<std_msgs::String> mode_;

			/// Indicates whether to update Jacobians during the update call
			bool update_jacobians_;

			///	Visual debug
			EParam<std_msgs::Bool> visual_debug_;
			ros::Publisher ps_pub_;
	};
	typedef boost::shared_ptr<Scene> Scene_ptr;
	typedef std::map<std::string, Scene_ptr> Scene_map;

}	//	namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
