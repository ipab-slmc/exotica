//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_SCENE_H_
#define EXOTICA_CORE_SCENE_H_

#include <fstream>
#include <functional>
#include <iostream>
#include <string>

#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>

#include <exotica_core/collision_scene.h>
#include <exotica_core/kinematic_tree.h>
#include <exotica_core/object.h>
#include <exotica_core/property.h>
#include <exotica_core/tools/conversions.h>
#include <exotica_core/trajectory.h>

#include <exotica_core/scene_initializer.h>

namespace exotica
{
struct AttachedObject
{
    AttachedObject() = default;
    AttachedObject(std::string _parent) : parent(_parent) {}
    AttachedObject(std::string _parent, KDL::Frame _pose) : parent(_parent), pose(_pose) {}
    std::string parent = "";
    KDL::Frame pose;
};

/// The class of EXOTica Scene
class Scene : public Object, Uncopyable, public Instantiable<SceneInitializer>
{
public:
    Scene(const std::string& name);
    Scene();
    virtual ~Scene();
    virtual void Instantiate(SceneInitializer& init);
    void RequestKinematics(KinematicsRequest& request, std::function<void(std::shared_ptr<KinematicResponse>)> callback);
    std::string GetName();
    virtual void Update(Eigen::VectorXdRefConst x, double t = 0);
    void SetCollisionScene(const moveit_msgs::PlanningScene& scene);
    CollisionScenePtr& GetCollisionScene();
    std::string GetRootFrameName();
    std::string GetRootJointName();
    moveit_msgs::PlanningScene GetPlanningSceneMsg();
    exotica::KinematicTree& GetKinematicTree();
    void GetJointNames(std::vector<std::string>& joints);
    std::vector<std::string> GetJointNames();
    std::vector<std::string> GetModelJointNames();
    std::vector<std::string> GetControlledLinkNames();
    std::vector<std::string> GetModelLinkNames();
    Eigen::VectorXd GetControlledState();
    Eigen::VectorXd GetModelState();
    std::map<std::string, double> GetModelStateMap();
    void SetModelState(Eigen::VectorXdRefConst x, double t = 0, bool update_traj = true);
    void SetModelState(std::map<std::string, double> x, double t = 0, bool update_traj = true);
    std::string GetGroupName();

    void AddTrajectoryFromFile(const std::string& link, const std::string& traj);
    void AddTrajectory(const std::string& link, const std::string& traj);
    void AddTrajectory(const std::string& link, std::shared_ptr<Trajectory> traj);
    std::shared_ptr<Trajectory> GetTrajectory(const std::string& link);
    void RemoveTrajectory(const std::string& link);

    /// \brief Updates exotica scene object frames from the MoveIt scene.
    void UpdateSceneFrames();
    ///
    /// \brief Attaches existing object to a new parent. E.g. attaching a grasping target to the end-effector. The relative transformation will be computed from current object and new parent transformations in the world frame.
    /// \param name Name of the object to attach.
    /// \param parent Name of the new parent frame.
    ///
    void AttachObject(const std::string& name, const std::string& parent);
    ///
    /// \brief Attaches existing object to a new parent specifying an offset in the new parent local frame.
    /// \param name Name of the object to attach.
    /// \param parent Name of the new parent frame.
    /// \param pose Relative pose of the attached object in the new parent's local frame.
    ///
    void AttachObjectLocal(const std::string& name, const std::string& parent, const KDL::Frame& pose);
    ///
    /// \brief Detaches an object and leaves it a at its current world location. This effectively attaches the object to the world frame.
    /// \param name Name of the object to detach.
    ///
    void DetachObject(const std::string& name);
    bool HasAttachedObject(const std::string& name);

    void AddObject(const std::string& name, const KDL::Frame& transform = KDL::Frame(), const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), bool update_collision_scene = true);

    void AddObject(const std::string& name, const KDL::Frame& transform = KDL::Frame(), const std::string& parent = "", const std::string& shape_resource_path = "", Eigen::Vector3d scale = Eigen::Vector3d::Ones(), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), bool update_collision_scene = true);

    void AddObjectToEnvironment(const std::string& name, const KDL::Frame& transform = KDL::Frame(), shapes::ShapeConstPtr shape = nullptr, const Eigen::Vector4d& colour = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const bool& update_collision_scene = true);

    void RemoveObject(const std::string& name);

    /// @brief Update the internal MoveIt planning scene from a
    /// moveit_msgs::PlanningSceneWorld
    /// @param[in] world moveit_msgs::PlanningSceneWorld
    void UpdateWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world);

    void UpdateCollisionObjects();

    BaseType GetBaseType()
    {
        return base_type_;
    }

    void UpdateTrajectoryGenerators(double t = 0);

    void PublishScene();
    void PublishProxies(const std::vector<CollisionProxy>& proxies);
    visualization_msgs::Marker ProxyToMarker(const std::vector<CollisionProxy>& proxies, const std::string& frame);
    void LoadScene(const std::string& scene, const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity(), bool update_collision_scene = true);
    void LoadScene(const std::string& scene, const KDL::Frame& offset = KDL::Frame(), bool update_collision_scene = true);
    void LoadSceneFile(const std::string& file_name, const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity(), bool update_collision_scene = true);
    void LoadSceneFile(const std::string& file_name, const KDL::Frame& offset = KDL::Frame(), bool update_collision_scene = true);
    std::string GetScene();
    void CleanScene();

    /// @brief Whether the collision scene transforms get updated on every scene update.
    /// @return Whether collision scene transforms are force updated on every scene update.
    bool AlwaysUpdatesCollisionScene() { return force_collision_; }
    /// @brief Returns a map between a model link name and the names of associated collision links.
    /// @return Map between model links and all associated collision links.
    std::map<std::string, std::vector<std::string>> GetModelLinkToCollisionLinkMap() { return modelLink_to_collisionLink_map_; };
    /// @brief Returns a map between a model link name and the KinematicElement of associated collision links.
    /// @return Map between model links and all the KinematicElements of the associated collision links.
    std::map<std::string, std::vector<std::shared_ptr<KinematicElement>>> GetModelLinkToCollisionElementMap() { return modelLink_to_collisionElement_map_; };
    /// @brief Returns a map between controlled robot link names and associated collision link names. Here we consider all fixed links between controlled links as belonging to the previous controlled link (as if the collision links had been fused).
    /// @return Map between controlled links and associated collision links.
    std::map<std::string, std::vector<std::string>> GetControlledLinkToCollisionLinkMap() { return controlledLink_to_collisionLink_map_; };
private:
    void UpdateInternalFrames(bool update_request = true);

    /// @brief      Updates the internal state of the MoveIt PlanningScene from Kinematica.
    void UpdateMoveItPlanningScene();

    void LoadSceneFromStringStream(std::istream& in, const Eigen::Isometry3d& offset, bool update_collision_scene);

    /// The name of the scene
    std::string name_ = "Unnamed";

    /// The kinematica tree
    exotica::KinematicTree kinematica_;

    /// Joint group
    robot_model::JointModelGroup* group;

    /// Robot base type
    BaseType base_type_;

    /// The collision scene
    CollisionScenePtr collision_scene_;

    /// Internal MoveIt planning scene
    planning_scene::PlanningScenePtr ps_;

    /// Visual debug
    ros::Publisher ps_pub_;
    ros::Publisher proxy_pub_;

    /// \brief List of attached objects
    /// These objects will be reattached if the scene gets reloaded.
    std::map<std::string, AttachedObject> attached_objects_;

    /// \brief List of frames/links added on top of robot links and scene objects defined in the MoveIt scene.
    std::vector<std::shared_ptr<KinematicElement>> custom_links_;

    std::map<std::string, std::pair<std::weak_ptr<KinematicElement>, std::shared_ptr<Trajectory>>> trajectory_generators_;

    bool force_collision_;

    /// \brief Mapping between model link names and collision links.
    std::map<std::string, std::vector<std::string>> modelLink_to_collisionLink_map_;
    std::map<std::string, std::vector<std::shared_ptr<KinematicElement>>> modelLink_to_collisionElement_map_;

    /// \brief Mapping between controlled link name and collision links
    std::map<std::string, std::vector<std::string>> controlledLink_to_collisionLink_map_;

    /// \brief List of links to be excluded from the collision scene
    std::set<std::string> robotLinksToExcludeFromCollisionScene_;

    KinematicsRequest kinematic_request_;
    std::shared_ptr<KinematicResponse> kinematic_solution_;
    std::function<void(std::shared_ptr<KinematicResponse>)> kinematic_request_callback_;
    bool request_needs_updating_;
};

typedef std::shared_ptr<Scene> ScenePtr;
}

#endif  // EXOTICA_CORE_SCENE_H_
