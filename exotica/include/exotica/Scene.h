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
  typedef std::vector<std::shared_ptr<fcl::CollisionObject> > fcls_ptr;

///	The class of collision scene
  class CollisionScene : public Uncopyable
  {
    public:

      CollisionScene(const std::string & scene_name);

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
          const std::vector<std::string> & joints, const std::string & mode,
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
       * @return True, if the state is collision free.
       */
      bool isStateValid(const Eigen::VectorXd &q);

      /**
       * \brief Check for the closest distance in the planning scene including both self- and world-collisions
       * @return The closest distance - negative values indicate penetration.
       */
      double getClosestDistance(bool computeSelfCollisionDistance = true, bool debug = false);

      /**
       * \brief	Get closest distance between robot link and any other objects
       * @param	link	Robot link
       * @param	self	Indicate if self collision check is required
       * @param	d		Closest distance
       * @param	p1		Closest distance point on the link
       * @param	p2		Closest distance point on the other object
       * @param	norm	Normal vector on robot link
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

      /**
       * @brief      Update the internal MoveIt planning scene from a
       * moveit_msgs::PlanningSceneWorld
       *
       * @param[in]  world  moveit_msgs::PlanningSceneWorld
       */
      void updateWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world);
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

      std::string scene_name_;
      BASE_TYPE BaseType;

  };

  typedef std::shared_ptr<CollisionScene> CollisionScene_ptr;

///	The class of EXOTica Scene
  class Scene: public Object, Uncopyable, public Instantiable<SceneInitializer>
  {
    public:
      Scene(const std::string & name);
      Scene();
      virtual ~Scene();
      virtual void Instantiate(SceneInitializer& init);
      std::shared_ptr<KinematicResponse> RequestKinematics(KinematicsRequest& Request);
      std::string getName();
      virtual void Update(Eigen::VectorXdRefConst x);
      void setCollisionScene(const planning_scene::PlanningSceneConstPtr & scene);
      void setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene);
      int getNumJoints();
      CollisionScene_ptr & getCollisionScene();
      std::string getRootName();
      planning_scene::PlanningScenePtr getPlanningScene();
      exotica::KinematicTree & getSolver();
      robot_model::RobotModelPtr getRobotModel();
      void getJointNames(std::vector<std::string> & joints);
      std::vector<std::string> getJointNames();
      std::vector<std::string> getModelJointNames();
      Eigen::VectorXd getModelState();
      Eigen::VectorXd getControlledState();
      std::map<std::string, double> getModelStateMap();
      void setModelState(Eigen::VectorXdRefConst x);
      void setModelState(std::map<std::string, double> x);
      std::string getGroupName();

      BASE_TYPE getBaseType()
      {
        return BaseType;
      }

      void publishScene();
      void loadScene(const std::string& scene);
      void loadSceneFile(const std::string& file_name);
      std::string getScene();
      void cleanScene();
    private:
      ///	The name of the scene
      std::string name_;

      ///	The kinematica tree
      exotica::KinematicTree kinematica_;

      ///	Robot model
      robot_model::RobotModelPtr model_;

      ///   Joint group
      robot_model::JointModelGroup* group;

      ///	Robot base type
      BASE_TYPE BaseType;

      ///	The collision scene
      CollisionScene_ptr collision_scene_;

      /// Visual debug
      ros::Publisher ps_pub_;
  };
  typedef std::shared_ptr<Scene> Scene_ptr;
//  typedef std::map<std::string, Scene_ptr> Scene_map;

}	//	namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
