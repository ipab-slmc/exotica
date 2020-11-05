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

#ifndef EXOTICA_CORE_DYNAMICS_SOLVER_H_
template <typename T, int NX, int NU>
class AbstractDynamicsSolver;
typedef AbstractDynamicsSolver<double, Eigen::Dynamic, Eigen::Dynamic> DynamicsSolver;
#endif

/// The class of EXOTica Scene
class Scene : public Object, Uncopyable, public Instantiable<SceneInitializer>, public std::enable_shared_from_this<Scene>
{
public:
    Scene(const std::string& name);
    Scene();
    virtual ~Scene();
    virtual void Instantiate(const SceneInitializer& init);
    void RequestKinematics(KinematicsRequest& request, std::function<void(std::shared_ptr<KinematicResponse>)> callback);
    const std::string& GetName() const;  // Deprecated - use GetObjectName
    void Update(Eigen::VectorXdRefConst x, double t = 0);

    /// \brief Returns a pointer to the CollisionScene
    const CollisionScenePtr& GetCollisionScene() const;

    /// \brief Returns a pointer to the CollisionScene
    std::shared_ptr<DynamicsSolver> GetDynamicsSolver() const;

    std::string GetRootFrameName();
    std::string GetRootJointName();

    exotica::KinematicTree& GetKinematicTree();
    std::vector<std::string> GetControlledJointNames();
    std::vector<std::string> GetModelJointNames();
    std::vector<std::string> GetControlledLinkNames();
    std::vector<std::string> GetModelLinkNames();
    Eigen::VectorXd GetControlledState();
    Eigen::VectorXd GetModelState();
    std::map<std::string, double> GetModelStateMap();
    std::map<std::string, std::weak_ptr<KinematicElement>> GetTreeMap();
    void SetModelState(Eigen::VectorXdRefConst x, double t = 0, bool update_traj = true);
    void SetModelState(const std::map<std::string, double>& x, double t = 0, bool update_traj = true);

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
    void AttachObjectLocal(const std::string& name, const std::string& parent, const Eigen::VectorXd& pose);
    ///
    /// \brief Detaches an object and leaves it a at its current world location. This effectively attaches the object to the world frame.
    /// \param name Name of the object to detach.
    ///
    void DetachObject(const std::string& name);
    bool HasAttachedObject(const std::string& name);

    void AddObject(const std::string& name, const KDL::Frame& transform = KDL::Frame(), const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const bool update_collision_scene = true);

    void AddObject(const std::string& name, const KDL::Frame& transform = KDL::Frame(), const std::string& parent = "", const std::string& shape_resource_path = "", const Eigen::Vector3d& scale = Eigen::Vector3d::Ones(), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const bool update_collision_scene = true);

    void AddObjectToEnvironment(const std::string& name, const KDL::Frame& transform = KDL::Frame(), shapes::ShapeConstPtr shape = nullptr, const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const bool update_collision_scene = true);

    void RemoveObject(const std::string& name);

    /// @brief Update the collision scene from a moveit_msgs::PlanningSceneWorld
    /// @param[in] world moveit_msgs::PlanningSceneWorld
    void UpdatePlanningSceneWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world);

    /// @brief Update the collision scene from a moveit_msgs::PlanningScene
    /// @param[in] scene moveit_msgs::PlanningScene
    void UpdatePlanningScene(const moveit_msgs::PlanningScene& scene);

    /// @brief Returns the current robot configuration and collision environment
    /// as a moveit_msgs::PlanningScene
    moveit_msgs::PlanningScene GetPlanningSceneMsg();

    void UpdateCollisionObjects();
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
    bool AlwaysUpdatesCollisionScene() const { return force_collision_; }
    /// @brief Returns a map between a model link name and the names of associated collision links.
    /// @return Map between model links and all associated collision links.
    const std::map<std::string, std::vector<std::string>>& GetModelLinkToCollisionLinkMap() const { return model_link_to_collision_link_map_; };
    /// @brief Returns a map between a model link name and the KinematicElement of associated collision links.
    /// @return Map between model links and all the KinematicElements of the associated collision links.
    const std::map<std::string, std::vector<std::shared_ptr<KinematicElement>>>& GetModelLinkToCollisionElementMap() const { return model_link_to_collision_element_map_; };
    /// @brief Returns a map between controlled robot joint names and associated collision link names. Here we consider all fixed links between controlled links as belonging to the previous controlled joint (as if the collision links had been fused).
    /// @return Map between controlled joints and associated collision links.
    const std::map<std::string, std::vector<std::string>>& GetControlledJointToCollisionLinkMap() const { return controlled_joint_to_collision_link_map_; };
    /// @brief Returns world links that are to be excluded from collision checking.
    const std::set<std::string>& get_world_links_to_exclude_from_collision_scene() const { return world_links_to_exclude_from_collision_scene_; }
    int get_num_positions() const;
    int get_num_velocities() const;
    int get_num_controls() const;
    int get_num_state() const;
    int get_num_state_derivative() const;
    bool get_has_quaternion_floating_base() const;

private:
    bool has_quaternion_floating_base_ = false;  ///< Whether the state includes a SE(3) floating base.

    void UpdateInternalFrames(bool update_request = true);

    /// @brief      Updates the internal state of the MoveIt PlanningScene from Kinematica.
    void UpdateMoveItPlanningScene();

    void LoadSceneFromStringStream(std::istream& in, const Eigen::Isometry3d& offset, bool update_collision_scene);

    /// The kinematica tree
    exotica::KinematicTree kinematica_;

    /// The collision scene
    CollisionScenePtr collision_scene_;

    /// The dynamics solver
    std::shared_ptr<DynamicsSolver> dynamics_solver_ = std::shared_ptr<DynamicsSolver>(nullptr);

    int num_positions_ = 0;         ///< "nq"
    int num_velocities_ = 0;        ///< "nv"
    int num_controls_ = 0;          ///< "nu"
    int num_state_ = 0;             ///< "nx"
    int num_state_derivative_ = 0;  ///< "ndx"

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
    std::map<std::string, std::vector<std::string>> model_link_to_collision_link_map_;
    std::map<std::string, std::vector<std::shared_ptr<KinematicElement>>> model_link_to_collision_element_map_;

    /// \brief Mapping between controlled joint name and collision links
    std::map<std::string, std::vector<std::string>> controlled_joint_to_collision_link_map_;

    /// \brief List of robot links to be excluded from the collision scene
    std::set<std::string> robot_links_to_exclude_from_collision_scene_;

    /// \brief List of world links to be excluded from the collision scene
    std::set<std::string> world_links_to_exclude_from_collision_scene_;

    KinematicsRequest kinematic_request_;
    std::shared_ptr<KinematicResponse> kinematic_solution_;
    std::function<void(std::shared_ptr<KinematicResponse>)> kinematic_request_callback_;
    bool request_needs_updating_;
};

typedef std::shared_ptr<Scene> ScenePtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_SCENE_H_
