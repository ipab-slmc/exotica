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
#include <exotica/CollisionScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <exotica/Trajectory.h>

namespace exotica
{
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
      virtual void Update(Eigen::VectorXdRefConst x, double t = 0);
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
      void setModelState(Eigen::VectorXdRefConst x, double t = 0);
      void setModelState(std::map<std::string, double> x, double t = 0);
      std::string getGroupName();

      void addTrajectoryFromFile(const std::string& link, const std::string& traj);
      void addTrajectory(const std::string& link, const std::string& traj);
      void addTrajectory(const std::string& link, std::shared_ptr<Trajectory> traj);
      std::shared_ptr<Trajectory> getTrajectory(const std::string& link);
      void removeTrajectory(const std::string& link);

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

      void addObject(const std::string& name, const KDL::Frame& transform = KDL::Frame(), const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), bool updateCollisionScene = true);


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

      void updateTrajectoryGenerators(double t = 0);

      void publishScene();
      void publishProxies(const std::vector<CollisionProxy>& proxies);
      visualization_msgs::Marker proxyToMarker(const std::vector<CollisionProxy>& proxies, const std::string& frame);
      void loadScene(const std::string& scene, bool updaetCollisionScene = true);
      void loadSceneFile(const std::string& file_name, bool updaetCollisionScene = true);
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

      /// Internal moveit planning scene
      planning_scene::PlanningScenePtr ps_;

      /// Visual debug
      ros::Publisher ps_pub_;
      ros::Publisher proxy_pub_;

      /// \brief List of attached objects
      /// These objects will be reattached if the scene gets reloaded.
      std::map<std::string, AttachedObject> attached_objects_;

      /// \brief List of frames/links added on top of robot links and scene objects defined in the MoveIt scene.
      std::map<std::string, std::shared_ptr<KinematicElement>> custom_links_;

      std::map<std::string, std::pair<std::shared_ptr<KinematicElement>, std::shared_ptr<Trajectory>>> trajectory_generators_;

      bool force_collision_;
  };
  typedef std::shared_ptr<Scene> Scene_ptr;
//  typedef std::map<std::string, Scene_ptr> Scene_map;

} //  namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
