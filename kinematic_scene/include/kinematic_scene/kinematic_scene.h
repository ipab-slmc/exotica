/*
 * kinematic_scene.h
 *
 *  Created on: 12 Mar 2014
 *      Author: yiming
 */

#ifndef KINEMATIC_SCENE_H_
#define KINEMATIC_SCENE_H_

#include "tinyxml2/tinyxml2.h"
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <Eigen/Eigen>
#include <string>
#include <kinematica/KinematicTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_kdl.h>
#include <map>
#define INF 9999
namespace kinematica
{

	/** Kinematic Scene Initialisation Form */
	struct KinematicSceneForm_t
	{
			kinematica::SolutionForm_t optimisation;			//!< Kinematica solution form
			std::map<std::string, std::string> eff_world_map;//!< End-effector to world object mapping
			std::vector<std::string> ext_ids;//!< External object names that might be used (e.g. iMesh)
	};

	enum KS_EDIT_TYPE
	{
		ADD_WORLD = 0, ADD_ROBOT = 1, REMOVE_WORLD = 2, REMOVE_ROBOT = 2
	};
	struct KS_Object
	{
			std::string name;	//!< Object name
			std::string ref;	//!< Reference frame
			geometry_msgs::Pose origin;	//!< Origin of the object
			shape_msgs::SolidPrimitive shape;	//!< Object type
			KS_EDIT_TYPE type;	//!< Edit action type

	};

	class KinematicScene
	{
		public:
			/**
			 * @brief	Constructor of Kinematic Scene
			 * @param	name				Kinematic scene name
			 * @param	robot_description	Robot description
			 * @param	scene_srv			Moveit scene server name
			 * @param	scene_name			The name of our own planning scene
			 */
			KinematicScene(bool useSpinner = false);
			KinematicScene(const std::string & name, bool useSpinner = false);
			KinematicScene(const std::string & name, const std::string & robot_description,
					const std::string & scene_srv, const std::string & scene_name, bool useSpinner =
							false);

			virtual ~KinematicScene()
			{
			}
			;

			/**
			 * @brief	Initialisation of KS
			 * @param	urdf_file	Path to robot urdf file
			 * @param	ks_form		KS initialisation form
			 * @param	handle		Initialisation info from xml
			 */
			bool initKinematicScene(const std::string & urdf_file,
					const KinematicSceneForm_t & ks_form);
			bool initKinematicScene(tinyxml2::XMLHandle & handle);

			/**
			 * @brief	Get the name of this kinematic scene
			 * @return	Name of this kinematic scene
			 */
			std::string getName();

			/**
			 * @brief	Update the configuration of the robot
			 * @return	True if succeeded, false otherwise
			 */
            bool update(const Eigen::Ref<const Eigen::VectorXd> & joint_configuration, const int t);

			/**
			 * @brief	Get Forward Map (Wrap both joints and external objects, the object forward position
			 * 			will not be included in the result if the transform is unknown, and its name will be
			 * 			listed in the unknown objects)
			 * @param	phi				Forward Map
			 * @param	unkonwn_objects	Objects that unknown to the scene
			 * @return	True if succeeded, false otherwise
			 */
			bool getForwardMap(Eigen::Ref<Eigen::VectorXd> phi,
					std::vector<std::string> & unknown_objects);

			/**
			 * @brief	Get Jacobian
			 * @param	jac		Jacobian matrix
			 * @return	True if succeeded, false otherwise
			 */
			bool getJacobian(Eigen::Ref<Eigen::MatrixXd> jac);

			/**
			 * @brief	Get Poses of world objects
			 * @param	names	Names of the objects (can be robot links, or world objects)
			 * @param	poses	Poses of the objects
			 * @return	True if all poses are available, false otherwise
			 */
			bool getPoses(const std::vector<std::string> names, std::vector<KDL::Frame> & poses);

			/**
			 * @brief	Add new objects to kinematic scene
			 * @param	object	KS_objects
			 * @return	True if succeeded, false otherwise
			 */
			bool addObject(const KS_Object object);

			/**
			 * @brief	We may need to publish things to the scene,
			 * 			if we dont have a real robot or simulation running and want to see the motion
			 */
			void startPublishing();
			void stopPublishing();
			void publishScene();

			/**
			 * @brief	Since we are using moveit scene, we need to update to the newest state before each planning
			 * @param	scene	The planning scene from moveit
			 */
			bool updateScene(const planning_scene::PlanningSceneConstPtr & scene);
			bool updateScene();

			/**
			 * \brief Returns the number of kinematic maps this scene computes.
			 * @return Number of maps
			 */
			int getMapSize();

            std::string getRootName();

			/**
			 * @brief	Get the planning scene
			 * @return	Planning scene
			 */
			planning_scene::PlanningScenePtr getPlanningScene();

			/**
			 * @brief	Get the robot tree
			 * @return	Kinematica tree
			 */
			kinematica::KinematicTree getRobotTree();

			/**
			 * @brief	Distance to collision
			 * @return	Closest distance to collision
			 */
			double distanceToCollision();

			/**
			 * @brief	Check if the scene has been initialised
			 * @return	True if initialised
			 */
			bool isInitialised();

			/**
			 * @brief	Get the kinematica solver
			 * @return	Kinematica tree
			 */
			kinematica::KinematicTree & getSolver();

			/**
			 * @brief	Get initial end-effector
			 * @param	segs	Inital end-effector segment names
			 * @param	offsets	Inital end-effector offsets
			 */
			bool getInitialEff(std::vector<std::string> & segs, std::vector<KDL::Frame> & offsets);

			/**
			 * \brief	Get centre of mass properties
			 * @param	seg		Segment names
			 * @param	mass	Mass of each link
			 * @param	cog		Centre of gravity of each link
			 * @param	tip_pose	Tip pose of each link
			 * @param	base_pose	Base pose of each link
			 */
			bool getCoMProperties(std::vector<std::string> & segs, Eigen::VectorXd & mass,
					std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
					std::vector<KDL::Frame> & base_pose);

			bool updateEndEffectors(kinematica::SolutionForm_t & tmp_sol);

			/**
			 * \brief	Get AllowedCollisionMatrix
			 * @return AllowedCollisionMatrix
			 */
			const  collision_detection::AllowedCollisionMatrix& getACM() const;
			bool initialised_; //!< Initialisation flag

            /**
             * @brief getNumJoints
             * @return Number of controllable joints of the undrlying kinematic tree
             */
            int getNumJoints();

		private:

			void sceneCallback(const moveit_msgs::PlanningScene::ConstPtr & scene);

			/**
			 * @brief	Update Scene to Kinematica (update attached object offsets)
			 */
			bool updateSceneToKinematica();

			void initSrv(const std::string & scene_srv);
			void KS_INFO(std::string info);
			std::string name_;	//!< The name of this kinematic scene
			ros::NodeHandle nh_;	//!< Internal ros node handle
			boost::scoped_ptr<ros::AsyncSpinner> spinner_;	//!< Spinner
			ros::Publisher scene_pub_;	//!< Planning scene publisher
			ros::ServiceClient get_scene_client_;//!< Action client for the asking moveit planning scene
			moveit_msgs::GetPlanningScene get_scene_srv_;//!< Action server for the asking moveit planning scene
			kinematica::KinematicTree kinematic_solver_; /** Kinematica Solver */
			planning_scene::PlanningScenePtr ps_;	//!< Internal planning scene
			std::map<std::string, std::string> eff_world_map_;//!< Internally store the end-effectors to world objects map
			std::string task_root_;	//!< The task root
			std::vector<std::string> ext_ids_;	//!< Names of external objects
			boost::mutex locker_;	//!< Mutex
			bool isPublishing_;	//!< Publishing scene flag
			bool useSpinner_; //!< True if the spinner is used, false otherwise. With EXOTica planner manager, the spinner is not needed anymore.
	};
	typedef boost::shared_ptr<kinematica::KinematicScene> KinematicScene_ptr;
	typedef std::map<std::string, KinematicScene_ptr> KinematicScene_map; //!< Has to be a pointer so that we can use boost shared pointers

}

#endif /* KINEMATIC_SCENE_H_ */
