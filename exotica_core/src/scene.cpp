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

#include <eigen_conversions/eigen_kdl.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <kdl_conversions/kdl_msg.h>

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>
#include <exotica_core/server.h>
#include <exotica_core/setup.h>

#include <exotica_core/attach_link_initializer.h>
#include <exotica_core/box_shape_initializer.h>
#include <exotica_core/collision_scene_initializer.h>
#include <exotica_core/cylinder_shape_initializer.h>
#include <exotica_core/link_initializer.h>
#include <exotica_core/mesh_shape_initializer.h>
#include <exotica_core/octree_shape_initializer.h>
#include <exotica_core/shape_initializer.h>
#include <exotica_core/sphere_shape_initializer.h>
#include <exotica_core/trajectory_initializer.h>

#include <moveit/version.h>

namespace exotica
{
Scene::Scene() = default;

Scene::Scene(const std::string& name) : request_needs_updating_(false)
{
    object_name_ = name;
}

Scene::~Scene() = default;

const std::string& Scene::GetName() const
{
    return object_name_;
}

void Scene::Instantiate(const SceneInitializer& init)
{
    Object::InstantiateObject(SceneInitializer(init));
    this->parameters_ = init;
    kinematica_.debug = debug_;

    // Load robot model and set up kinematics (KinematicTree)
    robot_model::RobotModelPtr model;
    if (init.URDF == "" || init.SRDF == "")
    {
        Server::Instance()->GetModel(init.RobotDescription, model);
    }
    else
    {
        Server::Instance()->GetModel(init.URDF, model, init.URDF, init.SRDF);
    }
    kinematica_.Instantiate(init.JointGroup, model, object_name_);
    ps_.reset(new planning_scene::PlanningScene(model));

    // Write URDF/SRDF to ROS param server
    if (Server::IsRos() && init.SetRobotDescriptionRosParams && init.URDF != "" && init.SRDF != "")
    {
        if (debug_) HIGHLIGHT_NAMED(object_name_, "Setting robot_description and robot_description_semantic from URDF and SRDF initializers");
        const std::string urdf_string = PathExists(init.URDF) ? LoadFile(init.URDF) : init.URDF;
        const std::string srdf_string = PathExists(init.SRDF) ? LoadFile(init.SRDF) : init.SRDF;
        Server::SetParam("/robot_description", urdf_string);
        Server::SetParam("/robot_description_semantic", srdf_string);
    }

    // Set up debug topics if running in ROS mode
    if (Server::IsRos())
    {
        ps_pub_ = Server::Advertise<moveit_msgs::PlanningScene>(object_name_ + (object_name_ == "" ? "" : "/") + "PlanningScene", 1, true);
        proxy_pub_ = Server::Advertise<visualization_msgs::Marker>(object_name_ + (object_name_ == "" ? "" : "/") + "CollisionProxies", 1, true);
        if (debug_)
            HIGHLIGHT_NAMED(object_name_, "Running in debug mode, planning scene will be published to '" << Server::Instance()->GetName() << "/" << object_name_ << "/PlanningScene'");
    }

    // Note: Using the LoadScene initializer does not support custom offsets/poses, assumes Identity transform to world_frame
    if (init.LoadScene != "")
    {
        std::vector<std::string> files = ParseList(init.LoadScene, ';');
        for (const std::string& file : files) LoadSceneFile(file, Eigen::Isometry3d::Identity(), false);
    }

    // Add custom links
    for (const exotica::Initializer& linkInit : init.Links)
    {
        LinkInitializer link(linkInit);

        shapes::ShapePtr link_shape = nullptr;
        Eigen::Vector4d link_color = Eigen::Vector4d::Zero();
        if (link.Shape.size() == 1)
        {
            ShapeInitializer shape(link.Shape[0]);
            link_color = shape.Color;

            if (shape.Type == "Box")
            {
                BoxShapeInitializer box(link.Shape[0]);
                std::cout << "BOX: " << box.Dimensions.transpose() << std::endl;
                link_shape.reset(new shapes::Box(box.Dimensions.x(), box.Dimensions.y(), box.Dimensions.z()));
            }
            else if (shape.Type == "Cylinder")
            {
                CylinderShapeInitializer cylinder(link.Shape[0]);
                link_shape.reset(new shapes::Cylinder(cylinder.Radius, cylinder.Length));
            }
            else if (shape.Type == "Mesh")
            {
                MeshShapeInitializer mesh(link.Shape[0]);
                // TODO: This will not support textures.
                link_shape.reset(shapes::createMeshFromResource(ParsePath(mesh.MeshFilePath), mesh.Scale));
            }
            else if (shape.Type == "Octree")
            {
                OctreeShapeInitializer octree(link.Shape[0]);
                link_shape = LoadOctreeAsShape(ParsePath(octree.OctreeFilePath));
            }
            else if (shape.Type == "Sphere")
            {
                SphereShapeInitializer sphere(link.Shape[0]);
                link_shape.reset(new shapes::Sphere(sphere.Radius));
            }
            else
            {
                ThrowPretty("Unrecognized ShapeType: " << shape.Type);
            }
        }
        else if (link.Shape.size() > 1)
        {
            ThrowPretty("Only one Shape per Link allowed, given: " << link.Shape.size());
        }

        AddObject(link.Name, GetFrame(link.Transform), link.Parent, link_shape, KDL::RigidBodyInertia(link.Mass, GetFrame(link.CenterOfMass).p), link_color, false);  // false since CollisionScene is not yet setup
    }

    // Check list of robot links to exclude from CollisionScene
    if (init.RobotLinksToExcludeFromCollisionScene.size() > 0)
    {
        for (const auto& link : init.RobotLinksToExcludeFromCollisionScene)
        {
            robot_links_to_exclude_from_collision_scene_.insert(link);
            if (debug_) HIGHLIGHT_NAMED("RobotLinksToExcludeFromCollisionScene", link);
        }
    }

    // Check list of world links to exclude from CollisionScene
    if (init.WorldLinksToExcludeFromCollisionScene.size() > 0)
    {
        for (const auto& link : init.WorldLinksToExcludeFromCollisionScene)
        {
            world_links_to_exclude_from_collision_scene_.insert(link);
            if (debug_) HIGHLIGHT_NAMED("WorldLinksToExcludeFromCollisionScene", link);
        }
    }

    // Initialize CollisionScene
    force_collision_ = init.AlwaysUpdateCollisionScene;
    if (!init.DoNotInstantiateCollisionScene)
    {
        if (init.CollisionScene.size() == 0)
        {
            // Not set ==> Default to CollisionSceneFCLLatest for backwards compatibility.
            collision_scene_ = Setup::CreateCollisionScene("CollisionSceneFCLLatest");  // NB: This is an implicit run-time dependency and thus dangerous! But we don't want to break existing configs...
        }
        else if (init.CollisionScene.size() == 1)
        {
            collision_scene_ = Setup::CreateCollisionScene(init.CollisionScene[0]);
        }
        else
        {
            // Only one dynamics solver per scene is allowed, i.e., throw if more than one provided:
            ThrowPretty("Only one CollisionScene per scene allowed - " << init.CollisionScene.size() << " provided");
        }

        collision_scene_->debug_ = this->debug_;  // Backwards compatibility - TODO: Remove
        // collision_scene_->ns_ = ns_ + "/" + collision_scene_->GetObjectName();
        collision_scene_->Setup();  // TODO: Trigger from Initializer
        collision_scene_->AssignScene(shared_from_this());
        collision_scene_->SetAlwaysExternallyUpdatedCollisionScene(force_collision_);
    }

    UpdateSceneFrames();
    UpdateInternalFrames(false);

    // Attach links
    for (const exotica::Initializer& linkInit : init.AttachLinks)
    {
        AttachLinkInitializer link(linkInit);
        if (link.Local)
        {
            AttachObjectLocal(link.Name, link.Parent, GetFrame(link.Transform));
        }
        else
        {
            AttachObject(link.Name, link.Parent);
        }
    }

    // Set up allowed collision matrix (i.e., collision pairs that are filtered/ignored)
    AllowedCollisionMatrix acm;
    std::vector<std::string> acm_names;
    ps_->getAllowedCollisionMatrix().getAllEntryNames(acm_names);
    for (auto& name1 : acm_names)
    {
        for (auto& name2 : acm_names)
        {
            collision_detection::AllowedCollision::Type type = collision_detection::AllowedCollision::Type::ALWAYS;
            ps_->getAllowedCollisionMatrix().getAllowedCollision(name1, name2, type);
            if (type == collision_detection::AllowedCollision::Type::ALWAYS)
            {
                acm.setEntry(name1, name2);
            }
        }
    }
    if (collision_scene_ != nullptr) collision_scene_->SetACM(acm);

    // Set up trajectory generators
    for (const exotica::Initializer& it : init.Trajectories)
    {
        TrajectoryInitializer trajInit(it);
        if (trajInit.File != "")
        {
            AddTrajectoryFromFile(trajInit.Link, trajInit.File);
        }
        else
        {
            AddTrajectory(trajInit.Link, trajInit.Trajectory);
        }
    }

    // Set up DynamicsSolver (requires a fully initialised scene)
    if (init.DynamicsSolver.size() > 0)
    {
        // Only one dynamics solver per scene is allowed, i.e., throw if more than one provided:
        if (init.DynamicsSolver.size() > 1) ThrowPretty("Only one DynamicsSolver per scene allowed - " << init.DynamicsSolver.size() << " provided");

        // Create dynamics solver
        dynamics_solver_ = Setup::CreateDynamicsSolver(init.DynamicsSolver.at(0));
        dynamics_solver_->AssignScene(shared_from_this());
        dynamics_solver_->ns_ = ns_ + "/" + dynamics_solver_->GetObjectName();

        num_positions_ = dynamics_solver_->get_num_positions();
        num_velocities_ = dynamics_solver_->get_num_velocities();
        num_controls_ = dynamics_solver_->get_num_controls();
        num_state_ = dynamics_solver_->get_num_state();
        num_state_derivative_ = dynamics_solver_->get_num_state_derivative();
    }
    else
    {
        num_positions_ = GetKinematicTree().GetNumModelJoints();
        num_velocities_ = 0;
        num_controls_ = 0;
        num_state_ = num_positions_;
        num_state_derivative_ = GetKinematicTree().GetNumControlledJoints();
    }

    // Check if the system has a floating-base, and if so, if it contains a quaternion.
    // Will need to trigger special logic below to handle this (w.r.t. normalisation).
    has_quaternion_floating_base_ = (GetKinematicTree().GetModelBaseType() == BaseType::FLOATING && num_state_ == num_state_derivative_ + 1);

    if (debug_) INFO_NAMED(object_name_, "Exotica Scene initialized");
}

void Scene::RequestKinematics(KinematicsRequest& request, std::function<void(std::shared_ptr<KinematicResponse>)> callback)
{
    kinematic_request_ = request;
    kinematic_request_callback_ = callback;
    kinematic_solution_ = kinematica_.RequestFrames(kinematic_request_);
    kinematic_request_callback_(kinematic_solution_);
    request_needs_updating_ = false;
}

void Scene::UpdateTrajectoryGenerators(double t)
{
    for (auto& it : trajectory_generators_)
    {
        it.second.first.lock()->generated_offset = it.second.second->GetPosition(t);
    }
}

void Scene::Update(Eigen::VectorXdRefConst x, double t)
{
    if (request_needs_updating_ && kinematic_request_callback_)
    {
        UpdateInternalFrames();
    }

    UpdateTrajectoryGenerators(t);
    kinematica_.Update(x);
    if (force_collision_ && collision_scene_ != nullptr) collision_scene_->UpdateCollisionObjectTransforms();
    if (debug_) PublishScene();
}

void Scene::UpdateMoveItPlanningScene()
{
    std::map<std::string, double> modelState = GetModelStateMap();
    for (const auto& joint : modelState)
    {
        try
        {
            ps_->getCurrentStateNonConst().setVariablePosition(joint.first, joint.second);
        }
        catch (const std::out_of_range& e)
        {
            HIGHLIGHT("Could not find Kinematica joint name in MoveIt: " + joint.first);
        }
    }

    // The floating base joint in MoveIt uses quaternion, while Kinematica uses
    // RPY [but using rot_x, rot_y, and rot_z as joint names]. Thus, we need to
    // fix the orientation of the virtual floating base by extracting the RPY
    // values, converting them to quaternion, and then updating the planning
    // scene.
    if (kinematica_.GetModelBaseType() == BaseType::FLOATING)
    {
        KDL::Rotation rot = KDL::Rotation::RPY(modelState[kinematica_.GetRootJointName() + "/rot_x"], modelState[kinematica_.GetRootJointName() + "/rot_y"], modelState[kinematica_.GetRootJointName() + "/rot_z"]);
        Eigen::Quaterniond quat(Eigen::Map<const Eigen::Matrix3d>(rot.data).transpose());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.GetRootJointName() + "/rot_x", quat.x());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.GetRootJointName() + "/rot_y", quat.y());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.GetRootJointName() + "/rot_z", quat.z());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.GetRootJointName() + "/rot_w", quat.w());
    }
}

void Scene::PublishScene()
{
    if (Server::IsRos())
    {
        ps_pub_.publish(GetPlanningSceneMsg());
    }
}

void Scene::PublishProxies(const std::vector<CollisionProxy>& proxies)
{
    if (Server::IsRos())
    {
        proxy_pub_.publish(ProxyToMarker(proxies, kinematica_.GetRootFrameName()));
    }
}

visualization_msgs::Marker Scene::ProxyToMarker(const std::vector<CollisionProxy>& proxies, const std::string& frame)
{
    visualization_msgs::Marker ret;
    ret.header.frame_id = "exotica/" + frame;
    ret.action = visualization_msgs::Marker::ADD;
    ret.frame_locked = false;
    ret.ns = "Proxies";
    ret.color.a = 1.0;
    ret.id = 0;
    ret.type = visualization_msgs::Marker::LINE_LIST;
    ret.points.resize(proxies.size() * 6);
    ret.colors.resize(proxies.size() * 6);
    ret.scale.x = 0.005;
    double normalLength = 0.01;
    std_msgs::ColorRGBA normal = GetColor(0.8, 0.8, 0.8);
    std_msgs::ColorRGBA far = GetColor(0.5, 0.5, 0.5);
    std_msgs::ColorRGBA colliding = GetColor(1, 0, 0);
    for (int i = 0; i < proxies.size(); ++i)
    {
        KDL::Vector c1 = KDL::Vector(proxies[i].contact1(0), proxies[i].contact1(1), proxies[i].contact1(2));
        KDL::Vector c2 = KDL::Vector(proxies[i].contact2(0), proxies[i].contact2(1), proxies[i].contact2(2));
        KDL::Vector n1 = KDL::Vector(proxies[i].normal1(0), proxies[i].normal1(1), proxies[i].normal1(2));
        KDL::Vector n2 = KDL::Vector(proxies[i].normal2(0), proxies[i].normal2(1), proxies[i].normal2(2));
        tf::pointKDLToMsg(c1, ret.points[i * 6]);
        tf::pointKDLToMsg(c2, ret.points[i * 6 + 1]);
        tf::pointKDLToMsg(c1, ret.points[i * 6 + 2]);
        tf::pointKDLToMsg(c1 + n1 * normalLength, ret.points[i * 6 + 3]);
        tf::pointKDLToMsg(c2, ret.points[i * 6 + 4]);
        tf::pointKDLToMsg(c2 + n2 * normalLength, ret.points[i * 6 + 5]);
        ret.colors[i * 6] = ret.colors[i * 6 + 1] = proxies[i].distance > 0 ? far : colliding;
        ret.colors[i * 6 + 2] = ret.colors[i * 6 + 3] = ret.colors[i * 6 + 4] = ret.colors[i * 6 + 5] = normal;
    }
    return ret;
}

void Scene::UpdatePlanningScene(const moveit_msgs::PlanningScene& scene)
{
    ps_->usePlanningSceneMsg(scene);
    UpdateSceneFrames();
    UpdateInternalFrames();
}

void Scene::UpdatePlanningSceneWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world)
{
    ps_->processPlanningSceneWorldMsg(*world);
    UpdateSceneFrames();
    UpdateInternalFrames();
}

void Scene::UpdateCollisionObjects()
{
    if (collision_scene_ != nullptr) collision_scene_->UpdateCollisionObjects(kinematica_.GetCollisionTreeMap());
}

const CollisionScenePtr& Scene::GetCollisionScene() const
{
    if (collision_scene_ == nullptr) ThrowPretty("No CollisionScene initialized!");
    return collision_scene_;
}

std::shared_ptr<DynamicsSolver> Scene::GetDynamicsSolver() const
{
    return dynamics_solver_;
}

std::string Scene::GetRootFrameName()
{
    return kinematica_.GetRootFrameName();
}

std::string Scene::GetRootJointName()
{
    return kinematica_.GetRootJointName();
}

moveit_msgs::PlanningScene Scene::GetPlanningSceneMsg()
{
    // Update the joint positions in the PlanningScene from Kinematica - we do
    // not do this on every Update() as it is only required when publishing
    // the scene and would take unnecessary time otherwise.
    UpdateMoveItPlanningScene();

    moveit_msgs::PlanningScene msg;
    ps_->getPlanningSceneMsg(msg);

    // The robot link paddings and scales are applied in the CollisionScene
    // and are not propagated back to the internal MoveIt PlanningScene. Here
    // we set them in the message that is being returned.
    msg.link_padding.clear();
    msg.link_scale.clear();
    for (auto robot_link : msg.robot_state.joint_state.name)
    {
        moveit_msgs::LinkPadding padding;
        padding.link_name = robot_link;
        padding.padding = (collision_scene_ != nullptr) ? collision_scene_->GetRobotLinkPadding() : 0.0;
        msg.link_padding.push_back(padding);

        moveit_msgs::LinkScale scale;
        scale.link_name = robot_link;
        scale.scale = (collision_scene_ != nullptr) ? collision_scene_->GetRobotLinkScale() : 1.0;
        msg.link_scale.push_back(scale);
    }

    // As we cannot apply world link scalings in the message itself, we need to
    // manually scale the objects.
    // TODO(wxm): Recreate as updated poses won't be reflected (e.g. trajectories)
    if (collision_scene_ != nullptr && (collision_scene_->GetWorldLinkScale() != 1.0 || collision_scene_->GetWorldLinkPadding() > 0.0))
    {
        for (auto it : msg.world.collision_objects)
        {
            // Primitives
            for (auto primitive : it.primitives)
            {
                shapes::ShapePtr tmp(shapes::constructShapeFromMsg(primitive));
                tmp->scaleAndPadd(collision_scene_->GetWorldLinkScale(), collision_scene_->GetWorldLinkPadding());
                shapes::ShapeMsg tmp_msg;
                shapes::constructMsgFromShape(const_cast<shapes::Shape*>(tmp.get()), tmp_msg);
                primitive = boost::get<shape_msgs::SolidPrimitive>(tmp_msg);
            }

            // Meshes
            for (auto mesh : it.meshes)
            {
                shapes::ShapePtr tmp(shapes::constructShapeFromMsg(mesh));
                tmp->scaleAndPadd(collision_scene_->GetWorldLinkScale(), collision_scene_->GetWorldLinkPadding());
                shapes::ShapeMsg tmp_msg;
                shapes::constructMsgFromShape(const_cast<shapes::Shape*>(tmp.get()), tmp_msg);
                mesh = boost::get<shape_msgs::Mesh>(tmp_msg);
            }

            // NB: Scaling and padding does not apply to planes
        }
    }

    return msg;
}

exotica::KinematicTree& Scene::GetKinematicTree()
{
    return kinematica_;
}

std::vector<std::string> Scene::GetControlledJointNames()
{
    return kinematica_.GetControlledJointNames();
}

std::vector<std::string> Scene::GetControlledLinkNames()
{
    return kinematica_.GetControlledLinkNames();
}

std::vector<std::string> Scene::GetModelLinkNames()
{
    return kinematica_.GetModelLinkNames();
}

std::vector<std::string> Scene::GetModelJointNames()
{
    return kinematica_.GetModelJointNames();
}

Eigen::VectorXd Scene::GetModelState()
{
    return kinematica_.GetModelState();
}

std::map<std::string, double> Scene::GetModelStateMap()
{
    return kinematica_.GetModelStateMap();
}

std::map<std::string, std::weak_ptr<KinematicElement>> Scene::GetTreeMap()
{
    return kinematica_.GetTreeMap();
}

void Scene::SetModelState(Eigen::VectorXdRefConst x, double t, bool update_traj)
{
    if (request_needs_updating_ && kinematic_request_callback_)
    {
        UpdateInternalFrames();
    }

    if (update_traj) UpdateTrajectoryGenerators(t);
    // Update Kinematica internal state
    kinematica_.SetModelState(x);

    if (force_collision_ && collision_scene_ != nullptr) collision_scene_->UpdateCollisionObjectTransforms();
    if (debug_) PublishScene();
}

void Scene::SetModelState(const std::map<std::string, double>& x, double t, bool update_traj)
{
    if (request_needs_updating_ && kinematic_request_callback_)
    {
        UpdateInternalFrames();
    }

    if (update_traj) UpdateTrajectoryGenerators(t);
    // Update Kinematica internal state
    kinematica_.SetModelState(x);

    if (force_collision_ && collision_scene_ != nullptr) collision_scene_->UpdateCollisionObjectTransforms();
    if (debug_) PublishScene();
}

Eigen::VectorXd Scene::GetControlledState()
{
    return kinematica_.GetControlledState();
}

void Scene::LoadScene(const std::string& scene, const KDL::Frame& offset, bool update_collision_scene)
{
    Eigen::Isometry3d tmp_offset;
    tf::transformKDLToEigen(offset, tmp_offset);
    LoadScene(scene, tmp_offset, update_collision_scene);
}

void Scene::LoadScene(const std::string& scene, const Eigen::Isometry3d& offset, bool update_collision_scene)
{
    std::stringstream ss(scene);
    LoadSceneFromStringStream(ss, offset, update_collision_scene);
}

void Scene::LoadSceneFile(const std::string& file_name, const KDL::Frame& offset, bool update_collision_scene)
{
    Eigen::Isometry3d tmp_offset;
    tf::transformKDLToEigen(offset, tmp_offset);
    LoadSceneFile(file_name, tmp_offset, update_collision_scene);
}

void Scene::LoadSceneFile(const std::string& file_name, const Eigen::Isometry3d& offset, bool update_collision_scene)
{
    std::ifstream ss(ParsePath(file_name));
    if (!ss.is_open()) ThrowPretty("Cant read file '" << ParsePath(file_name) << "'!");
    LoadSceneFromStringStream(ss, offset, update_collision_scene);
}

void Scene::LoadSceneFromStringStream(std::istream& in, const Eigen::Isometry3d& offset, bool update_collision_scene)
{
#if ROS_VERSION_MINIMUM(1, 14, 0)  // if ROS version >= ROS_MELODIC
    ps_->loadGeometryFromStream(in, offset);
#else
    ps_->loadGeometryFromStream(in, Eigen::Affine3d(offset));
#endif

    UpdateSceneFrames();
    if (update_collision_scene) UpdateInternalFrames();
}

std::string Scene::GetScene()
{
    std::stringstream ss;
    ps_->saveGeometryToStream(ss);
    // TODO: include all custom environment scene objects
    return ss.str();
}

void Scene::CleanScene()
{
    ps_->removeAllCollisionObjects();
    // TODO: remove all custom environment scene objects
    UpdateSceneFrames();
}

void Scene::UpdateInternalFrames(bool update_request)
{
    for (auto& it : custom_links_)
    {
        Eigen::Isometry3d pose;
        tf::transformKDLToEigen(it->segment.getFrameToTip(), pose);
        std::string shape_resource_path = it->shape_resource_path;
        Eigen::Vector3d scale = it->scale;
        it = kinematica_.AddElement(it->segment.getName(), pose, it->parent_name, it->shape, it->segment.getInertia(), it->color, it->visual, it->is_controlled);
        it->shape_resource_path = shape_resource_path;
        it->scale = scale;
    }

    auto trajectory_generators_copy = trajectory_generators_;
    trajectory_generators_.clear();
    for (auto& traj : trajectory_generators_copy)
    {
        AddTrajectory(traj.first, traj.second.second);
    }

    for (auto& link : attached_objects_)
    {
        AttachObjectLocal(link.first, link.second.parent, link.second.pose);
    }

    kinematica_.UpdateModel();

    if (update_request)
    {
        kinematic_solution_ = kinematica_.RequestFrames(kinematic_request_);
        kinematic_request_callback_(kinematic_solution_);
    }

    UpdateCollisionObjects();

    request_needs_updating_ = false;
}

void Scene::UpdateSceneFrames()
{
    kinematica_.ResetModel();

    // Add world objects
    std::map<std::string, int> visual_map;
    for (const auto& object : *ps_->getWorld())
    {
        if (object.second->shapes_.size())
        {
            // Use the first collision shape as the origin of the object
            Eigen::Isometry3d obj_transform;
            obj_transform.translation() = object.second->shape_poses_[0].translation();
            obj_transform.linear() = object.second->shape_poses_[0].rotation();
            std::shared_ptr<KinematicElement> element = kinematica_.AddEnvironmentElement(object.first, obj_transform);
            std::vector<VisualElement> visuals;
            for (int i = 0; i < object.second->shape_poses_.size(); ++i)
            {
                Eigen::Isometry3d shape_transform;
                shape_transform.translation() = object.second->shape_poses_[i].translation();
                shape_transform.linear() = object.second->shape_poses_[i].rotation();
                Eigen::Isometry3d trans = obj_transform.inverse() * shape_transform;
                VisualElement visual;
                std::string name = object.first;
                // Check for name duplicates after loading from scene files
                const auto& it = visual_map.find(name);
                if (it != visual_map.end())
                {
                    it->second++;
                    name = name + "_" + std::to_string(it->second);
                }
                else
                {
                    visual_map[name] = 0;
                }
                visual.name = name;
                visual.shape = shapes::ShapePtr(object.second->shapes_[i]->clone());
                tf::transformEigenToKDL(trans, visual.frame);
                if (ps_->hasObjectColor(object.first))
                {
                    auto color_msg = ps_->getObjectColor(object.first);
                    visual.color = Eigen::Vector4d(color_msg.r, color_msg.g, color_msg.b, color_msg.a);
                    kinematica_.AddEnvironmentElement(object.first + "_collision_" + std::to_string(i), trans, object.first, object.second->shapes_[i], KDL::RigidBodyInertia::Zero(), visual.color);
                }
                else
                {
                    kinematica_.AddEnvironmentElement(object.first + "_collision_" + std::to_string(i), trans, object.first, object.second->shapes_[i]);
                }
                if (visual.shape) visuals.push_back(visual);
            }
            element->visual = visuals;
        }
        else
        {
            HIGHLIGHT("Object with no shapes ('" << object.first << "')");
        }
    }

    // Add robot collision objects
    ps_->getCurrentStateNonConst().update(true);
#if MOVEIT_VERSION_MAJOR >= 1 && MOVEIT_VERSION_MINOR >= 1
    const std::vector<const robot_model::LinkModel*>& links = ps_->getCollisionEnv()->getRobotModel()->getLinkModelsWithCollisionGeometry();
#else
    const std::vector<const robot_model::LinkModel*>& links = ps_->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();
#endif

    int last_controlled_joint_id = -1;
    std::string last_controlled_joint_name = "";
    model_link_to_collision_link_map_.clear();
    model_link_to_collision_element_map_.clear();
    controlled_joint_to_collision_link_map_.clear();
    for (int i = 0; i < links.size(); ++i)
    {
        // Check whether link is excluded from collision checking
        if (robot_links_to_exclude_from_collision_scene_.find(links[i]->getName()) != robot_links_to_exclude_from_collision_scene_.end())
        {
            continue;
        }

        Eigen::Isometry3d obj_transform;
        obj_transform.translation() = ps_->getCurrentState().getGlobalLinkTransform(links[i]).translation();
        obj_transform.linear() = ps_->getCurrentState().getGlobalLinkTransform(links[i]).rotation();

        int joint_id = GetKinematicTree().IsControlledLink(links[i]->getName());
        if (joint_id != -1)
        {
            if (last_controlled_joint_id != joint_id)
            {
                last_controlled_joint_name = GetKinematicTree().GetControlledJointNames()[joint_id];
                last_controlled_joint_id = joint_id;
            }
        }

        for (int j = 0; j < links[i]->getShapes().size(); ++j)
        {
            // // Workaround for issue #172
            // // To disable collisions with visual tags, we define a sphere of radius 0 which we will ignore here
            // // This is in contrast to IHMC's convention where a sphere of radius 0 in the SDF is a desired contact point
            // if (links[i]->getShapes()[j]->type == shapes::ShapeType::SPHERE)
            //     if (static_cast<const shapes::Sphere*>(links[i]->getShapes()[j].get())->radius == 0.0)
            //         continue;

            Eigen::Affine3d collisionBodyTransform_affine = ps_->getCurrentState().getCollisionBodyTransform(links[i], j);
            Eigen::Isometry3d collisionBodyTransform;
            collisionBodyTransform.translation() = collisionBodyTransform_affine.translation();
            collisionBodyTransform.linear() = collisionBodyTransform_affine.rotation();
            Eigen::Isometry3d trans = obj_transform.inverse(Eigen::Isometry) * collisionBodyTransform;

            std::string collision_element_name = links[i]->getName() + "_collision_" + std::to_string(j);
            std::shared_ptr<KinematicElement> element = kinematica_.AddElement(collision_element_name, trans, links[i]->getName(), links[i]->getShapes()[j]);
            model_link_to_collision_element_map_[links[i]->getName()].push_back(element);

            // Set up mappings
            model_link_to_collision_link_map_[links[i]->getName()].push_back(collision_element_name);

            if (last_controlled_joint_name != "")
            {
                controlled_joint_to_collision_link_map_[last_controlled_joint_name].push_back(collision_element_name);
            }
        }
    }

    kinematica_.UpdateModel();

    request_needs_updating_ = true;
}

void Scene::AddObject(const std::string& name, const KDL::Frame& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, bool update_collision_scene)
{
    if (kinematica_.DoesLinkWithNameExist(name)) ThrowPretty("Link '" << name << "' already exists in the scene!");
    std::string parent_name = (parent == "") ? kinematica_.GetRootFrameName() : parent;
    if (!kinematica_.DoesLinkWithNameExist(parent_name)) ThrowPretty("Can't find parent '" << parent_name << "'!");
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    custom_links_.push_back(kinematica_.AddElement(name, pose, parent_name, shape, inertia, color));
    if (update_collision_scene) UpdateCollisionObjects();
}

void Scene::AddObject(const std::string& name, const KDL::Frame& transform, const std::string& parent, const std::string& shape_resource_path, const Eigen::Vector3d& scale, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, bool update_collision_scene)
{
    if (kinematica_.DoesLinkWithNameExist(name)) ThrowPretty("Link '" << name << "' already exists in the scene!");
    std::string parent_name = (parent == "") ? kinematica_.GetRootFrameName() : parent;
    if (!kinematica_.DoesLinkWithNameExist(parent_name)) ThrowPretty("Can't find parent '" << parent_name << "'!");
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    custom_links_.push_back(kinematica_.AddElement(name, pose, parent_name, shape_resource_path, scale, inertia, color));
    UpdateSceneFrames();
    UpdateInternalFrames();
    if (update_collision_scene) UpdateCollisionObjects();
}

void Scene::AddObjectToEnvironment(const std::string& name, const KDL::Frame& transform, shapes::ShapeConstPtr shape, const Eigen::Vector4d& color, const bool update_collision_scene)
{
    if (kinematica_.HasModelLink(name))
    {
        throw std::runtime_error("link '" + name + "' already exists in kinematic tree");
    }
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    ps_->getWorldNonConst()->addToObject(name, shape, pose);
    ps_->setObjectColor(name, GetColor(color));
    UpdateSceneFrames();
    if (update_collision_scene) UpdateInternalFrames();
}

void Scene::RemoveObject(const std::string& name)
{
    auto it = std::begin(custom_links_);
    while (it != std::end(custom_links_))
    {
        if ((*it)->segment.getName() == name)
        {
            custom_links_.erase(it);
            UpdateSceneFrames();
            UpdateInternalFrames();
            return;
        }
        else
        {
            ++it;
        }
    }
    ThrowPretty("Link " << name << " not removed as it cannot be found.");
}

void Scene::AttachObject(const std::string& name, const std::string& parent)
{
    kinematica_.ChangeParent(name, parent, KDL::Frame::Identity(), false);
    attached_objects_[name] = AttachedObject(parent);
}

void Scene::AttachObjectLocal(const std::string& name, const std::string& parent, const KDL::Frame& pose)
{
    kinematica_.ChangeParent(name, parent, pose, true);
    attached_objects_[name] = AttachedObject(parent, pose);
}

void Scene::AttachObjectLocal(const std::string& name, const std::string& parent, const Eigen::VectorXd& pose)
{
    AttachObjectLocal(name, parent, GetFrame(pose));
}

void Scene::DetachObject(const std::string& name)
{
    if (!HasAttachedObject(name)) ThrowPretty("'" << name << "' is not attached to the robot!");
    auto object = attached_objects_.find(name);
    kinematica_.ChangeParent(name, "", KDL::Frame::Identity(), false);
    attached_objects_.erase(object);
}

bool Scene::HasAttachedObject(const std::string& name)
{
    return attached_objects_.find(name) != attached_objects_.end();
}

void Scene::AddTrajectoryFromFile(const std::string& link, const std::string& traj)
{
    AddTrajectory(link, LoadFile(traj));
}

void Scene::AddTrajectory(const std::string& link, const std::string& traj)
{
    AddTrajectory(link, std::shared_ptr<Trajectory>(new Trajectory(traj)));
}

void Scene::AddTrajectory(const std::string& link, std::shared_ptr<Trajectory> traj)
{
    const auto& tree = kinematica_.GetTreeMap();
    const auto& it = tree.find(link);
    if (it == tree.end()) ThrowPretty("Can't find link '" << link << "'!");
    if (traj->GetDuration() == 0.0) ThrowPretty("The trajectory is empty!");
    trajectory_generators_[link] = std::pair<std::weak_ptr<KinematicElement>, std::shared_ptr<Trajectory>>(it->second, traj);
    it->second.lock()->is_trajectory_generated = true;
}

std::shared_ptr<Trajectory> Scene::GetTrajectory(const std::string& link)
{
    const auto& it = trajectory_generators_.find(link);
    if (it == trajectory_generators_.end()) ThrowPretty("No trajectory generator defined for link '" << link << "'!");
    return it->second.second;
}

void Scene::RemoveTrajectory(const std::string& link)
{
    const auto& it = trajectory_generators_.find(link);
    if (it == trajectory_generators_.end()) ThrowPretty("No trajectory generator defined for link '" << link << "'!");
    it->second.first.lock()->is_trajectory_generated = false;
    trajectory_generators_.erase(it);
}

int Scene::get_num_positions() const
{
    return num_positions_;
}

int Scene::get_num_velocities() const
{
    return num_velocities_;
}

int Scene::get_num_controls() const
{
    return num_controls_;
}

int Scene::get_num_state() const
{
    return num_state_;
}

int Scene::get_num_state_derivative() const
{
    return num_state_derivative_;
}

bool Scene::get_has_quaternion_floating_base() const
{
    return has_quaternion_floating_base_;
}

}  // namespace exotica
