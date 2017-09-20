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

//  For collision
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/narrowphase/narrowphase.h>
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

/// The class of collision scene
  class CollisionScene : public Uncopyable
  {
    public:

      CollisionScene(const std::string & scene_name);

      /**
       * \brief Destructor
       */
      virtual ~CollisionScene();

      /**
       * \brief Initialisation function
       * @param psmsg Moveit planning scene message
       * @param joints  Joint names
       * @param mode  Update mode
       * @return Indication of success
       */
      void initialise(const moveit_msgs::PlanningSceneConstPtr & psmsg,
          const std::vector<std::string> & joints, const std::string & mode,
          BASE_TYPE base_type, robot_model::RobotModelPtr model_);

      /**
       * \brief Update the robot collision properties
       * @param x   Configuration
       * @return Indication of success
       */
      void update(Eigen::VectorXdRefConst x);

      /**
       * @brief      Update an individual joint's value in the planning scene
       *
       * @param[in]  joint  The joint
       * @param[in]  value  The value
       */
      void update(std::string joint, double value);

      /**
       * \brief Get closest distance between two objects.
       * @param o1    Name of object 1
       * @param o2    Name of object 2
       * @param d   Signed closest distance (negative values indicate
       * penetration)
       */
      void getDistance(const std::string& o1, const std::string& o2, double& d,
                       double safeDist = 0.01);

      /**
       * @brief Get closest distance between two objects.
       * @param     o1        Name of object 1
       * @param     o2        Name of object 2
       * @param     d         Signed closest distance (negative values indicate
       * penetration)
       * @param     p1        If not in contact, closest point on o1. If in
       * contact, position of the contact in world frame.
       * @param     p2        If not in contact, closest point on o2. If in
       * contact, normalized contact normal.
       * @param[in] safeDist  Minimum distance to be considered safe, must be
       * above 0.
       */
      void getDistance(const std::string& o1, const std::string& o2, double& d,
                       Eigen::Vector3d& p1, Eigen::Vector3d& p2,
                       double safeDist = 0.01);

      /**
       * \brief Check if the whole robot is valid (collision only)
       * @param self  Indicate if self collision check is required
       * @return True, if the state is collision free.
       */
      bool isStateValid(bool self = true, double dist = 0);

      /**
       * \brief Get closest distance between robot link and any other objects.
       * @param link  Robot link
       * @param self  Indicate if self collision check is required
       * @param d   Closest distance. Returns -1 if in collision.
       * @param p1    Closest distance point on the link
       * @param p2    Closest distance point on the other object
       * @param norm  Normal vector on robot link (p2-p1)
       * @param c1  Center of the AABB of the colliding link
       * @param c2  Center of the AABB of the other object
       */
      void getRobotDistance(const std::string& link, bool self, double& d,
                            Eigen::Vector3d& p1, Eigen::Vector3d& p2,
                            Eigen::Vector3d& norm, Eigen::Vector3d& c1,
                            Eigen::Vector3d& c2, double safeDist);

      /**
       * \brief Get current robot state
       * @return  Current robot state
       */
      const robot_state::RobotState& getCurrentState();

      /**
       * \brief Get the moveit planning scene
       * @return  Moveit planning scene
       */
      planning_scene::PlanningScenePtr getPlanningScene();

      // Potentially deprecated
      inline std::map<std::string, fcls_ptr>& getFCLWorld()
      {
        return fcl_world_;
      }

      // Potentially deprecated
      inline std::map<std::string, fcls_ptr>& getFCLRobot()
      {
        return fcl_robot_;
      }

      /**
       * @brief      Gets the collision world links.
       *
       * @return     The collision world links.
       */
      std::vector<std::string> getCollisionWorldLinks() {
        std::vector<std::string> tmp;
        for (auto& it : fcl_world_) tmp.push_back(it.first);
        return tmp;
      }

      /**
       * @brief      Gets the collision robot links.
       *
       * @return     The collision robot links.
       */
      std::vector<std::string> getCollisionRobotLinks() {
        std::vector<std::string> tmp;
        for (auto& it : fcl_robot_) tmp.push_back(it.first);
        return tmp;
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

      /**
       * @brief      Reinitializes the FCL CollisionScene world links. Call this
       * function if you update the MoveIt planning scene.
       */
      void reinitializeCollisionWorld();

    private:
     /**
      * @brief      Reinitializes the FCL CollisionScene robot links. This
      * function is automatically called when the scene is initialized via
      * the initialise() method.
      */
     void reinitializeCollisionRobot();

     /**
      * @brief      Updates the transforms of the robot links in the FCL
      * CollisionScene. This function is automatically called by the update()
      * methods.
      */
     void updateCollisionRobot();

     /**
      * \brief Get closest distance between two fcl objects - this is the main
      * distance computation function called by the other methods. It will
      * return -1 if the objects are in collision.
      * @param fcl1  FCL object 1
      * @param fcl2  FCL object 2
      * @param req   FCL distance request
      * @param res   FCL distance result
      * @return Distance to collision (-1 if in collision)
      */
     double distance(const fcls_ptr& fcl1, const fcls_ptr& fcl2,
                     const fcl::DistanceRequest& req, fcl::DistanceResult& res,
                     double safeDist);

     /**
      * @brief      Calculates the contact information. If there are multiple
      * contacts between the two FCL objects, the information of the contact
      * with the most penetration is returned.
      *
      * @param[in]  fcl1               FCL object 1
      * @param[in]  fcl2               FCL object 2
      * @param[in]  req                FCL collision request
      * @param[in]  res                FCL distance result
      * @param      penetration_depth  Maximum penetration depth
      * @param      pos                Position of the contact in world frame
      * @param      norm               Normalized contact normal
      */
     void computeContact(const fcls_ptr& fcl1, const fcls_ptr& fcl2,
                         const fcl::CollisionRequest& req,
                         fcl::CollisionResult& res, double& penetration_depth,
                         Eigen::Vector3d& pos, Eigen::Vector3d& norm);

     /**
      * @brief      Gets the signed distance between two named objects. This
      * function is the internal/private wrapper function providing
      * functionality which is subsequently exposed via different overloads.
      *
      * @param[in]  o1                           Name of FCL link 1
      * @param[in]  o2                           Name of FCL link 2
      * @param      d                            Signed distance - negative
      * values indicate penetration
      * @param[in]  calculateContactInformation  Whether to calculate the
      * contact information, i.e. contact point and normals.
      * @param[in]  safeDist                     Safety distance, needs to be
      * greater than 0.
      * @param      p1                           Nearest point on link 1 or if
      * in contact, the position of the contact in world frame.
      * @param      p2                           Nearest point on link 2 or if
      * in contact, the normalized contact normal.
      */
     void getDistance(const std::string& o1, const std::string& o2, double& d,
                      const bool calculateContactInformation,
                      const double safeDist, Eigen::Vector3d& p1,
                      Eigen::Vector3d& p2);

     /// FCL collision object for the robot
     std::map<std::string, fcls_ptr> fcl_robot_;

     /// FCL collision object for the world
     std::map<std::string, fcls_ptr> fcl_world_;

     /// FCL collision geometry for the robot
     std::map<std::string, geos_ptr> geo_robot_;

     /// FCL collision geometry for the world
     std::map<std::string, geos_ptr> geo_world_;

     /// To correct FCL transform
     std::map<std::string, std::vector<fcl::Transform3f>> trans_world_;

     /// Internal moveit planning scene
     planning_scene::PlanningScenePtr ps_;

     /// Joint index in robot state
     std::vector<int> joint_index_;

     /// Indicate if distance computation is required
     bool compute_dist;

     /// The allowed collisiom matrix
     collision_detection::AllowedCollisionMatrixPtr acm_;

     std::string scene_name_;
     std::string world_joint_ = "";
     BASE_TYPE BaseType;
  };

  typedef std::shared_ptr<CollisionScene> CollisionScene_ptr;

  struct AttachedObject
  {
      AttachedObject() : Parent("") {}
      AttachedObject(std::string parent) : Parent(parent) {}
      AttachedObject(std::string parent, KDL::Frame pose) : Parent(parent), Pose(pose) {}
      std::string Parent;
      KDL::Frame Pose;
  };

/// The class of EXOTica Scene
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
      CollisionScene_ptr & getCollisionScene();
      std::string getRootFrameName();
      std::string getRootJointName();
      std::string getModelRootLinkName();
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

      /// \brief Updates exotica scene object frames from the MoveIt scene.
      void updateSceneFrames();
      ///
      /// \brief Attaches existing object to a new parent. E.g. attaching a grasping target to the end-effector. The relative transformation will be computed from current object and new parent transformations in the world frame.
      /// \param name Name of the object to attach.
      /// \param parent Name of the new parent frame.
      ///
      void attachObject(const std::string& name, const std::string& parent);
      ///
      /// \brief Attaches existing object to a new parent specifying an offset in the new parent local frame.
      /// \param name Name of the object to attach.
      /// \param parent Name of the new parent frame.
      /// \param pose Relative pose of the attached object in the new parent's local frame.
      ///
      void attachObjectLocal(const std::string& name, const std::string& parent, const KDL::Frame& pose);
      ///
      /// \brief Detaches an object and leaves it a at its current world location. This effectively attaches the object to the world frame.
      /// \param name Name of the object to detach.
      ///
      void detachObject(const std::string& name);
      bool hasAttachedObject(const std::string& name);

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
      /// The name of the scene
      std::string name_;

      /// The kinematica tree
      exotica::KinematicTree kinematica_;

      /// Robot model
      robot_model::RobotModelPtr model_;

      ///   Joint group
      robot_model::JointModelGroup* group;

      /// Robot base type
      BASE_TYPE BaseType;

      /// The collision scene
      CollisionScene_ptr collision_scene_;

      /// Visual debug
      ros::Publisher ps_pub_;

      std::map<std::string, AttachedObject> attached_objects_;
  };
  typedef std::shared_ptr<Scene> Scene_ptr;
//  typedef std::map<std::string, Scene_ptr> Scene_map;

} //  namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
