/*
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#ifndef EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_
#define EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_

#include "exotica/Object.h"
#include "exotica/Server.h"
#include "exotica/KinematicTree.h"
#include <exotica/Property.h>
#include <exotica/SceneInitializer.h>

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

///	The class of collision scene
  class CollisionScene : public Uncopyable
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
      void initialise(const moveit_msgs::PlanningSceneConstPtr & psmsg,
          const std::vector<std::string> & joints, std::string & mode,
          BASE_TYPE base_type, robot_model::RobotModelPtr model_);

      /**
       * \brief	Update the robot collision properties
       * @param	x		Configuration
       * @return Indication of success
       */
      void update(Eigen::VectorXdRefConst x);

      /**
       * \brief	Get closest distance between two objects
       * @param	o1		Name of object 1
       * @param	o2		Name of object 2
       * @param	d		Closest distance (-1 if in collision)
       * @return Indication of success
       */
      void getDistance(const std::string & o1, const std::string & o2,
          double& d, double safeDist);
      /**
       * @brief Get closest distance between two objects
       * @param	o1		Name of object 1
       * @param	o2		Name of object 2
       * @param	d		Closest distance (-1 if in collision)
       * @param	p1		Closest point on o1
       * @param	p2		Closest point on o2
       * @return Indication of success
       */
      void getDistance(const std::string & o1, const std::string & o2,
          double& d, Eigen::Vector3d & p1, Eigen::Vector3d & p2,
          double safeDist);

      /**
       * \brief	Check if the whole robot is valid (collision and feasibility)
       * @param	self	Indicate if self collision check is required
       * @return True, if the state is collision free.
       */
      bool isStateValid(bool self = true, double dist = 0);

      /**
       * \brief	Check if the whole robot is valid given a configuration
       * @param	q		The configuration to check
       * @param	self	Indicate if self collision check is required
       * @return True, if the state is collision free.
       */
      bool isStateValid(const Eigen::VectorXd &q, bool self = true);

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
      void getRobotDistance(const std::string & link, bool self, double & d,
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

      void updateWorld(
          const moveit_msgs::PlanningSceneWorldConstPtr & world);
      void getCollisionLinkTranslation(const std::string & name,
          Eigen::Vector3d & translation);
      void getWorldObjectTranslation(const std::string & name,
          Eigen::Vector3d & translation);
      void getTranslation(const std::string & name,
          Eigen::Vector3d & translation);
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
          const fcl::DistanceRequest & req, fcl::DistanceResult & res,
          double safeDist);
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
      collision_detection::AllowedCollisionMatrixPtr acm_;

      ///	Pointer to exotica server
      exotica::Server_ptr server_;
      std::string scene_name_;
      BASE_TYPE base_type_;
      EParam<std_msgs::Bool> drake_full_body_;;

  };

  typedef boost::shared_ptr<CollisionScene> CollisionScene_ptr;

///	The class of EXOTica Scene
  class Scene: public Object, Uncopyable, public Instantiable<SceneInitializer>
  {
    public:
      /**
       * \brief	Default constructor
       * @param	name	The scene name
       */
      Scene(const std::string & name);
      Scene();

      /**
       * \brief	Destructor
       */
      virtual ~Scene();

      virtual void Instantiate(SceneInitializer& init);

      std::shared_ptr<KinematicResponse> RequestKinematics(KinematicsRequest& Request);

      /**
       * \brief	Get scene name
       * @return	Name
       */
      std::string getName();

      /**
       * \brief	Updator function
       * @param	x	System state
       * @return Indication of success
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t = 0);
      virtual void Update(Eigen::VectorXdRefConst x);

      /**
       * \brief	Set collision scene
       * @param	scene	Moveit planning scene
       * @return Indication of success
       */
      void setCollisionScene(
          const planning_scene::PlanningSceneConstPtr & scene);

      /**
       * @brief Set collision scene
       * @param scene Planning scene message
       * @return Indication of success
       */
      void setCollisionScene(
          const moveit_msgs::PlanningSceneConstPtr & scene);

      /**
       * \brief	Append new taskmap
       * @param	name	Taskmap name
       * @param	eff		Endeffector names
       * @param	offset	Endeffector offsets
       * @return Indication of success
       */
      void appendTaskMap(const std::string & name,
          const std::vector<std::string> & eff,
          const std::vector<KDL::Frame> & offset);

      /**
       * \brief	Clear all the appended taskmaps
       */
      void clearTaskMap();

      /**
       * \brief	Called after appending
       * @return Indication of success
       */
      void activateTaskMaps();

      /**
       * \brief	Update task map (eff)
       * @param	task	Task name
       * @param	offset	Task end-effector offsets
       * @return Indication of success
       */
      void updateEndEffectors(const std::string & task,
          const std::vector<KDL::Frame> & offset);

      /*
       * \brief Updated individual end-effector
       * @param task  Task name
       * @param eff   End-effector name
       * @param offset  New end-effector offset
       */
      void updateEndEffector(const std::string &task, const std::string &eff,
          const KDL::Frame& offset);

      /**
       * \brief	Get forward map (values)
       * @param	task	Task name
       * @param	phi		Returned forward map
       * @return Indication of success
       */
      void getForwardMap(const std::string & task, Eigen::VectorXdRef phi);

      /**
       * @brief Get forward map reference
       * @param task Task name
       * @param phi Returned forward map reference
       * @return Indication of success
       */
      void getForwardMap(const std::string & task,
          Eigen::VectorXdRef_ptr& phi, bool force = false);

      /**
       * \brief	Get jacobian (values)
       * @param	task	Task name
       * @param	jac		Returned Jacobian
       * @return Indication of success
       */
      void getJacobian(const std::string & task, Eigen::MatrixXdRef jac);

      /**
       * @brief Get jacobian reference
       * @param task Task name
       * @param jac Returned Jacobian reference
       * @return Indication of success
       */
      void getJacobian(const std::string & task, Eigen::MatrixXdRef_ptr& jac,
          bool force = false);

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
      void getEndEffectors(const std::string & task,
          std::vector<std::string> & effs);
      void getEndEffectors(const std::string & task,
          std::pair<std::vector<std::string>, std::vector<KDL::Frame>> & effs);

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
      void getCoMProperties(std::string& task,
          std::vector<std::string> & segs, Eigen::VectorXd & mass,
          std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
          std::vector<KDL::Frame> & base_pose);

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
      exotica::KinematicTree & getSolver();

      /**
       * @brief Returns poses ofrequested end-effectors
       * @param names List of end-effector names
       * @param poses Returned poses
       * @return Indication of success
       */
      void getPoses(const std::vector<std::string> & names,
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
      void getJointNames(std::vector<std::string> & joints);
      std::vector<std::string> getJointNames();

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

      void publishScene();
    private:
      ///	EXOTica server
      Server_ptr server_;

      ///	The name of the scene
      std::string name_;

      ///	The kinematica tree
      exotica::KinematicTree kinematica_;

      ///	Robot model
      robot_model::RobotModelPtr model_;

      ///   Joint group
      robot_model::JointModelGroup* group;

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
      std::string mode_;

      /// Indicates whether to update Jacobians during the update call
      bool update_jacobians_;

      ///	Visual debug
      bool visual_debug_;
      ros::Publisher ps_pub_;
  };
  typedef boost::shared_ptr<Scene> Scene_ptr;
//  typedef std::map<std::string, Scene_ptr> Scene_map;

}	//	namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
