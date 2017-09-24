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
#include <unordered_set>
#include <unordered_map>

namespace exotica
{
  class Scene;

  class AllowedCollisionMatrix
  {
  public:
      AllowedCollisionMatrix();
      AllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
      void clear();
      bool hasEntry(const std::string& name) const;
      void setEntry(const std::string& name1, const std::string& name2);
      void getAllEntryNames(std::vector<std::string>& names) const;
      bool getAllowedCollision(const std::string& name1, const std::string& name2) const;
  private:
      std::unordered_map<std::string, std::unordered_set<std::string>> entries_;
  };

  struct CollisionProxy
  {
      CollisionProxy() : e1(nullptr), e2(nullptr), distance(0) {}
      KinematicElement* e1;
      KinematicElement* e2;
      Eigen::Vector3d contact1;
      Eigen::Vector3d normal1;
      Eigen::Vector3d contact2;
      Eigen::Vector3d normal2;
      double distance;

      std::string print() const
      {
          std::stringstream ss;
          if(e1 && e2)
          {
              ss<<"Proxy: '"<<e1->Segment.getName()<<"' - '"<<e2->Segment.getName()<<"', c1: "<<contact1.transpose()<<" c1: "<<contact2.transpose()<<" d: "<<distance;
          }
          else
          {
              ss<<"Proxy (empty)";
          }
          return ss.str();
      }
  };
/// The class of collision scene
  class CollisionScene : public Uncopyable
  {
    public:

      struct CollisionData
      {
          CollisionData(CollisionScene* scene) : Scene(scene), Done(false), Self(true) {}

          fcl::CollisionRequest Request;
          fcl::CollisionResult Result;
          CollisionScene* Scene;
          bool Done;
          bool Self;
      };

      struct DistanceData
      {
          DistanceData(CollisionScene* scene) : Scene(scene), Self(true), Distance{1e300} {}

          fcl::DistanceRequest Request;
          fcl::DistanceResult Result;
          CollisionScene* Scene;
          std::vector<CollisionProxy> Proxies;
          double Distance;
          bool Self;
      };

      struct ContactData
      {
          ContactData(CollisionScene* scene) : Scene(scene), Self(true), Distance{1e300} {}

          fcl::DistanceRequest DistanceRequest;
          fcl::DistanceResult DistanceResult;
          fcl::CollisionRequest Request;
          fcl::CollisionResult Result;
          CollisionScene* Scene;
          std::vector<CollisionProxy> Proxies;
          double Distance;
          bool Self;
      };

      CollisionScene(const std::string & scene_name, robot_model::RobotModelPtr model, const std::string& root_name);

      /**
       * \brief Destructor
       */
      virtual ~CollisionScene();

      visualization_msgs::Marker proxyToMarker(const std::vector<CollisionProxy>& proxies);

      static bool isAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionScene* scene);
      static bool collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data);
      static bool collisionCallbackDistance(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist);
      static bool collisionCallbackContacts(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist);

      /**
       * \brief Check if the whole robot is valid (collision only).
       * @param self Indicate if self collision check is required.
       * @return True, if the state is collision free..
       */
      bool isStateValid(bool self = true);

      ///
      /// \brief Computes collision distances.
      /// \param self Indicate if self collision check is required.
      /// \param computePenetrationDepth If set to true, accurate penetration depth is computed.
      /// \return Collision proximity objectects for all colliding pairs of objects.
      ///
      std::vector<CollisionProxy> getCollisionDistance(bool self, bool computePenetrationDepth = true);

      /**
       * \brief Get the moveit planning scene
       * @return  Moveit planning scene
       */
      planning_scene::PlanningScenePtr getPlanningScene();

      /**
       * @brief      Gets the collision world links.
       *
       * @return     The collision world links.
       */
      std::vector<std::string> getCollisionWorldLinks()
      {
          std::vector<std::string> tmp;
          for (fcl::CollisionObject* object : fcl_objects_)
          {
              KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
              if(!element->ClosestRobotLink)
              {
                  tmp.push_back(element->Segment.getName());
              }
          }
          return tmp;
      }

      /**
       * @brief      Gets the collision robot links.
       *
       * @return     The collision robot links.
       */
      std::vector<std::string> getCollisionRobotLinks()
      {
          std::vector<std::string> tmp;
          for (fcl::CollisionObject* object : fcl_objects_)
          {
              KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
              if(element->ClosestRobotLink)
              {
                  tmp.push_back(element->Segment.getName());
              }
          }
          return tmp;
      }

      Eigen::Vector3d getTranslation(const std::string & name);

      inline void setACM(const AllowedCollisionMatrix& acm)
      {
          acm_ = acm;
      }

      ///
      /// \brief Creates the collision scene from kinematic elements.
      /// \param objects Vector kinematic element pointers of collision objects.
      ///
      void updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects);

      ///
      /// \brief Updates collision object transformations from the kinematic tree.
      ///
      void updateCollisionObjectTransforms();

    private:

     static std::shared_ptr<fcl::CollisionObject> constructFclCollisionObject(std::shared_ptr<KinematicElement> element);

     std::map<std::string, std::shared_ptr<fcl::CollisionObject>> fcl_cache_;

     std::vector<fcl::CollisionObject*> fcl_objects_;

     /// Internal moveit planning scene
     planning_scene::PlanningScenePtr ps_;

     /// Indicate if distance computation is required
     bool compute_dist;

     std::string root_name_;

     /// The allowed collisiom matrix
     AllowedCollisionMatrix acm_;
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


      /**
       * @brief      Update the internal MoveIt planning scene from a
       * moveit_msgs::PlanningSceneWorld
       *
       * @param[in]  world  moveit_msgs::PlanningSceneWorld
       */
      void updateWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world);

      BASE_TYPE getBaseType()
      {
        return BaseType;
      }

      void publishScene();
      void publishProxies(const std::vector<CollisionProxy>& proxies);
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
      ros::Publisher proxy_pub_;

      std::map<std::string, AttachedObject> attached_objects_;
  };
  typedef std::shared_ptr<Scene> Scene_ptr;
//  typedef std::map<std::string, Scene_ptr> Scene_map;

} //  namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
