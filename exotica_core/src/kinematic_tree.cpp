//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <algorithm>
#include <iostream>
#include <queue>
#include <set>

#include <eigen_conversions/eigen_kdl.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model/robot_model.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <exotica_core/kinematic_tree.h>
#include <exotica_core/server.h>
#include <exotica_core/tools.h>

namespace exotica
{
KinematicResponse::KinematicResponse() = default;

KinematicResponse::KinematicResponse(KinematicRequestFlags _flags, int _size, int _n)
{
    flags = _flags;
    frame.resize(_size);
    Phi.resize(_size);
    if (flags & KIN_FK_VEL) Phi_dot.resize(_size);
    KDL::Jacobian Jzero(_n);
    Jzero.data.setZero();
    Hessian Hzero = Hessian::Constant(6, Eigen::MatrixXd::Zero(_n, _n));
    if (_flags & KIN_J) jacobian = ArrayJacobian::Constant(_size, Jzero);
    if (_flags & KIN_H) hessian = ArrayHessian::Constant(_size, Hzero);
    x.setZero(_n);
}

KinematicsRequest::KinematicsRequest() = default;

KinematicFrameRequest::KinematicFrameRequest() = default;

KinematicFrameRequest::KinematicFrameRequest(std::string _frame_A_link_name, KDL::Frame _frame_A_offset, std::string _frame_B_link_name, KDL::Frame _frame_B_offset) : frame_A_link_name(_frame_A_link_name), frame_A_offset(_frame_A_offset), frame_B_link_name(_frame_B_link_name), frame_B_offset(_frame_B_offset)
{
}

KinematicSolution::KinematicSolution() = default;

KinematicSolution::KinematicSolution(int _start, int _length) : start(_start), length(_length)
{
}

void KinematicSolution::Create(std::shared_ptr<KinematicResponse> solution)
{
    if (start < 0 || length < 0) ThrowPretty("Kinematic solution was not initialized!");
    new (&Phi) Eigen::Map<ArrayFrame>(solution->Phi.data() + start, length);
    new (&X) Eigen::Map<Eigen::VectorXd>(solution->x.data(), solution->x.rows());
    if (solution->flags & KIN_FK_VEL) new (&Phi_dot) Eigen::Map<ArrayTwist>(solution->Phi_dot.data() + start, length);
    if (solution->flags & KIN_J) new (&jacobian) Eigen::Map<ArrayJacobian>(solution->jacobian.data() + start, length);
    if (solution->flags & KIN_H) new (&hessian) Eigen::Map<ArrayHessian>(solution->hessian.data() + start, length);
}

int KinematicTree::GetNumControlledJoints() const
{
    return num_controlled_joints_;
}

int KinematicTree::GetNumModelJoints() const
{
    return num_joints_;
}

void KinematicTree::Instantiate(const std::string& joint_group, robot_model::RobotModelPtr model, const std::string& name)
{
    if (!model) ThrowPretty("No robot model provided!");
    model_joints_names_ = model->getVariableNames();
    name_ = name;

    // Check if no joint group name is given - if so, default to all joints.
    const robot_model::JointModelGroup* group = model->getJointModelGroup(joint_group);
    if (!group)
    {
        auto names = model->getJointModelGroupNames();

        // If a particular joint group is desired, but we may have a user error and will throw an exception.
        if (!joint_group.empty())
        {
            std::stringstream ss;
            ss << "Joint group '" << joint_group << "' not defined in the robot model. " << names.size() << " joint groups available";
            if (names.size() > static_cast<std::size_t>(0))
            {
                ss << ": ";
                for (const auto& joint_group_name : names)
                    ss << joint_group_name << ", ";
            }
            ThrowPretty(ss.str());
        }
        else
        {
            // The joint group was left empty: We default to all _active_ joints (no fixed, no mimic).
            // Alternatives: getJointModelNames, getVariableNames
            for (auto joint_model : model->getActiveJointModels())
                controlled_joints_names_.push_back(joint_model->getName());
        }
    }
    else
    {
        // Use the joints from the desired joint group.
        controlled_joints_names_ = group->getVariableNames();
    }

    model_ = model;
    KDL::Tree robot_kinematics;
    if (kdl_parser::treeFromUrdfModel(*model_->getURDF(), robot_kinematics))
    {
        BuildTree(robot_kinematics);
    }
    else
    {
        ThrowPretty("Can't load URDF model!");
    }

    if (Server::IsRos())
    {
        shapes_pub_ = Server::Advertise<visualization_msgs::MarkerArray>(name_ + (name_ == "" ? "" : "/") + "CollisionShapes", 1, true);
        octomap_pub_ = Server::Advertise<octomap_msgs::Octomap>(name_ + (name_ == "" ? "" : "/") + "OctoMap", 1, true);
        debug_scene_changed_ = true;
        // Clear scene
        visualization_msgs::MarkerArray msg;
        msg.markers.resize(1);
        msg.markers[0].action = 3;  // DELETE_ALL
        shapes_pub_.publish(msg);
    }
}

void KinematicTree::BuildTree(const KDL::Tree& robot_kinematics)
{
    model_tree_.clear();
    tree_.clear();
    tree_map_.clear();
    model_joints_map_.clear();
    model_link_names_.clear();
    controlled_link_names_.clear();

    // Handle the root joint
    const robot_model::JointModel* root_joint = model_->getRootJoint();
    root_joint_name_ = root_joint->getName();
    std::string world_frame_name;
    for (const srdf::Model::VirtualJoint& s : model_->getSRDF()->getVirtualJoints())
    {
        if (s.name_ == root_joint->getName())
        {
            world_frame_name = s.parent_frame_;
        }
    }
    // if (world_frame_name == "") ThrowPretty("Can't initialize root joint!");
    if (world_frame_name.empty())
    {
        world_frame_name = "world_frame";
        WARNING_NAMED("KinematicTree", "No virtual joint defined. Defaulting world frame to " << world_frame_name)
    }

    // Extract Root Inertial
    double root_mass = 0.0;
    KDL::Vector root_cog = KDL::Vector::Zero();
    auto& urdf_root_inertial = model_->getURDF()->getRoot()->inertial;
    if (urdf_root_inertial)
    {
        root_mass = urdf_root_inertial->mass;
        root_cog = KDL::Vector(urdf_root_inertial->origin.position.x,
                               urdf_root_inertial->origin.position.y,
                               urdf_root_inertial->origin.position.z);
        if (debug)
            HIGHLIGHT_NAMED("Root Inertial", "Mass: " << root_mass
                                                      << " kg - CoG: " << root_cog);
    }
    // TODO: Note, this does not set the rotational inertia component, i.e. the
    // inertial matrix would be wrong
    KDL::RigidBodyInertia root_inertial(root_mass, root_cog);

    // Add general world_frame joint
    model_tree_.push_back(std::make_shared<KinematicElement>(-1, nullptr, KDL::Segment(world_frame_name, KDL::Joint(root_joint->getName(), KDL::Joint::None))));
    if (root_joint->getType() == robot_model::JointModel::FIXED)
    {
        model_base_type_ = BaseType::FIXED;
    }
    else if (root_joint->getType() == robot_model::JointModel::FLOATING)
    {
        model_base_type_ = BaseType::FLOATING;
        model_tree_.resize(7);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::TransZ, KDL::Joint::RotZ, KDL::Joint::RotY, KDL::Joint::RotX};
        const std::vector<std::string> floating_base_suffix = {
            "/trans_x", "/trans_y", "/trans_z",
            "/rot_z", "/rot_y", "/rot_x"};
        for (size_t i = 0; i < 6; ++i)
        {
            model_tree_[i + 1] = std::make_shared<KinematicElement>(i, model_tree_[i], KDL::Segment(world_frame_name + floating_base_suffix[i], KDL::Joint(root_joint_name_ + floating_base_suffix[i], types[i])));
            model_tree_[i]->children.push_back(model_tree_[i + 1]);
        }

        // The floating base rotation is defined as xyzw quaternion in the robot
        // model, but we are following a RPY-fixed axis (YPR rotating axis)
        // virtual joint convention in exotica - thus delete the rot_w from the
        // list of joint names
        auto RotW = std::find(controlled_joints_names_.begin(), controlled_joints_names_.end(), root_joint->getVariableNames()[6]);
        if (RotW != controlled_joints_names_.end()) controlled_joints_names_.erase(RotW);
        RotW = std::find(model_joints_names_.begin(), model_joints_names_.end(), root_joint->getVariableNames()[6]);
        if (RotW != model_joints_names_.end()) model_joints_names_.erase(RotW);

        // NB: We do not want to swap the XYZ labels for the rotation joints
        // to not change the order in which joint and model state are stored.
    }
    else if (root_joint->getType() == robot_model::JointModel::PLANAR)
    {
        model_base_type_ = BaseType::PLANAR;
        model_tree_.resize(4);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY,
                                         KDL::Joint::RotZ};
        for (int i = 0; i < 3; ++i)
        {
            model_tree_[i + 1] = std::make_shared<KinematicElement>(
                i, model_tree_[i],
                KDL::Segment(
                    root_joint->getVariableNames()[i],
                    KDL::Joint(root_joint->getVariableNames()[i], types[i])));
            model_tree_[i]->children.push_back(model_tree_[i + 1]);
        }
    }
    else
    {
        ThrowPretty("Unsupported root joint type: " << root_joint->getTypeName());
    }

    AddElementFromSegmentMapIterator(robot_kinematics.getRootSegment(), *(model_tree_.end() - 1));

    // Set root inertial
    if (root_joint->getType() == robot_model::JointModel::FIXED)
    {
        model_tree_[2]->segment.setInertia(root_inertial);
    }
    else if (root_joint->getType() == robot_model::JointModel::FLOATING)
    {
        model_tree_[7]->segment.setInertia(root_inertial);
    }
    else if (root_joint->getType() == robot_model::JointModel::PLANAR)
    {
        model_tree_[4]->segment.setInertia(root_inertial);
    }

    for (auto element : model_tree_) tree_.push_back(element);

    UpdateModel();
    tree_state_.setZero();

    if (debug)
    {
        for (int i = 0; i < tree_.size() - 1; ++i)
            HIGHLIGHT_NAMED(
                "Tree", "Joint: " << tree_[i].lock()->segment.getJoint().getName() << " - Link: " << tree_[i].lock()->segment.getName()
                                  << ", mass: " << tree_[i].lock()->segment.getInertia().getMass()
                                  << ", CoM: " << tree_[i].lock()->segment.getInertia().getCOG());
    }

    num_joints_ = model_joints_names_.size();
    num_controlled_joints_ = controlled_joints_names_.size();
    state_size_ = num_controlled_joints_;  // TODO: We should probably deprecate state_size_
    if (num_controlled_joints_ < 1) ThrowPretty("No update joints specified!");
    controlled_joints_.resize(num_controlled_joints_);
    for (std::shared_ptr<KinematicElement> Joint : model_tree_)
    {
        Joint->is_robot_link = true;
        Joint->control_id = IsControlled(Joint);
        Joint->is_controlled = Joint->control_id >= 0;
        model_joints_map_[Joint->segment.getJoint().getName()] = Joint;
        model_link_names_.push_back(Joint->segment.getName());
        if (Joint->is_controlled)
        {
            controlled_joints_[Joint->control_id] = Joint;
            controlled_joints_map_[Joint->segment.getJoint().getName()] = Joint;
            controlled_link_names_.push_back(Joint->segment.getName());

            // The model_base_type_ defined above refers to the base type of the
            // overall robot model - not of the set of controlled joints. E.g. a
            // floating-base robot can have a scene defined where the
            // floating-base virtual joint is _not_ part of the planning
            // group/scene. Thus we need to establish the BaseType of the joint
            // group, the controlled_base_type_ - if a controlled joint corresponds
            // to a floating base joint, the controlled_base_type_ is the same as the
            // model_base_type_.
            if (Joint->segment.getJoint().getName().find(
                    root_joint->getName()) != std::string::npos)
                controlled_base_type_ = model_base_type_;
        }
    }
    model_tree_[0]->is_robot_link = false;

    joint_limits_ = Eigen::MatrixXd::Zero(num_controlled_joints_, 2);
    velocity_limits_ = Eigen::VectorXd::Zero(num_controlled_joints_);
    acceleration_limits_ = Eigen::VectorXd::Zero(num_controlled_joints_);
    ResetJointLimits();

    if (debug) HIGHLIGHT_NAMED("KinematicTree::BuildTree", "Number of controlled joints: " << num_controlled_joints_ << " - Number of model joints: " << num_joints_);

    // Create random distributions for state sampling
    generator_ = std::mt19937(rd_());

    // Add visual shapes
    const urdf::ModelInterfaceSharedPtr& urdf = model_->getURDF();
    std::vector<urdf::LinkSharedPtr> urdf_links;
    urdf->getLinks(urdf_links);
    for (urdf::LinkSharedPtr urdf_link : urdf_links)
    {
        if (urdf_link->visual_array.size() > 0)
        {
            std::shared_ptr<KinematicElement> element = tree_map_[urdf_link->name].lock();
            int i = 0;
            element->visual.reserve(urdf_link->visual_array.size());
            for (urdf::VisualSharedPtr urdf_visual : urdf_link->visual_array)
            {
                VisualElement visual;
                visual.name = urdf_link->name + "_visual_" + std::to_string(i);
                visual.frame = KDL::Frame(KDL::Rotation::Quaternion(urdf_visual->origin.rotation.x, urdf_visual->origin.rotation.y, urdf_visual->origin.rotation.z, urdf_visual->origin.rotation.w), KDL::Vector(urdf_visual->origin.position.x, urdf_visual->origin.position.y, urdf_visual->origin.position.z));
                if (urdf_visual->material)
                {
                    urdf::MaterialSharedPtr material = urdf_visual->material->name == "" ? urdf_visual->material : urdf->getMaterial(urdf_visual->material->name);
                    visual.color(0) = material->color.r;
                    visual.color(1) = material->color.g;
                    visual.color(2) = material->color.b;
                    visual.color(3) = material->color.a;
                }
                else
                {
                    if (debug)
                        HIGHLIGHT("No material for " << visual.name);
                }

                switch (urdf_visual->geometry->type)
                {
                    case urdf::Geometry::BOX:
                    {
                        std::shared_ptr<urdf::Box> box = std::static_pointer_cast<urdf::Box>(ToStdPtr(urdf_visual->geometry));
                        visual.shape = std::shared_ptr<shapes::Box>(new shapes::Box(box->dim.x, box->dim.y, box->dim.z));
                    }
                    break;
                    case urdf::Geometry::SPHERE:
                    {
                        std::shared_ptr<urdf::Sphere> sphere = std::static_pointer_cast<urdf::Sphere>(ToStdPtr(urdf_visual->geometry));
                        visual.shape = std::shared_ptr<shapes::Sphere>(new shapes::Sphere(sphere->radius));
                    }
                    break;
                    case urdf::Geometry::CYLINDER:
                    {
                        std::shared_ptr<urdf::Cylinder> cylinder = std::static_pointer_cast<urdf::Cylinder>(ToStdPtr(urdf_visual->geometry));
                        visual.shape = std::shared_ptr<shapes::Cylinder>(new shapes::Cylinder(cylinder->radius, cylinder->length));
                    }
                    break;
                    case urdf::Geometry::MESH:
                    {
                        std::shared_ptr<urdf::Mesh> mesh = std::static_pointer_cast<urdf::Mesh>(ToStdPtr(urdf_visual->geometry));
                        visual.shape_resource_path = mesh->filename;
                        visual.scale = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                        visual.shape = std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(mesh->filename));
                    }
                    break;
                    default:
                        ThrowPretty("Unknown geometry type: " << urdf_visual->geometry->type);
                }
                element->visual.push_back(visual);
                ++i;
            }
        }
    }
}

void KinematicTree::UpdateModel()
{
    root_ = tree_[0].lock();
    tree_state_.conservativeResize(tree_.size());
    for (std::weak_ptr<KinematicElement> joint : tree_)
    {
        tree_map_[joint.lock()->segment.getName()] = joint.lock();
    }
    debug_tree_.resize(tree_.size() - 1);
    UpdateTree();
    debug_scene_changed_ = true;
}

void KinematicTree::ResetModel()
{
    collision_tree_map_.clear();
    tree_map_.clear();
    environment_tree_.clear();
    tree_.resize(model_tree_.size());
    UpdateModel();
    debug_scene_changed_ = true;

    // Remove all CollisionShapes
    if (Server::IsRos())
    {
        visualization_msgs::Marker mrk;
        mrk.action = 3;  // visualization_msgs::Marker::DELETEALL; // NB: enum only defined in ROS-jacobian and newer, functionality still there
        marker_array_msg_.markers.push_back(mrk);
    }
}

void KinematicTree::ChangeParent(const std::string& name, const std::string& parent_name, const KDL::Frame& pose, bool relative)
{
    if (tree_map_.find(name) == tree_map_.end()) ThrowPretty("Attempting to attach unknown frame '" << name << "'!");
    std::shared_ptr<KinematicElement> child = tree_map_.find(name)->second.lock();
    if (child->id < model_tree_.size()) ThrowPretty("Can't re-attach robot link '" << name << "'!");
    if (child->shape) ThrowPretty("Can't re-attach collision shape without reattaching the object! ('" << name << "')");
    std::shared_ptr<KinematicElement> parent;
    if (parent_name == "")
    {
        if (tree_map_.find(root_->segment.getName()) == tree_map_.end()) ThrowPretty("Attempting to attach to unknown frame '" << root_->segment.getName() << "'!");
        parent = tree_map_.find(root_->segment.getName())->second.lock();
    }
    else
    {
        if (tree_map_.find(parent_name) == tree_map_.end()) ThrowPretty("Attempting to attach to unknown frame '" << parent_name << "'!");
        parent = tree_map_.find(parent_name)->second.lock();
    }
    if (parent->shape) ThrowPretty("Can't attach object to a collision shape object! ('" << parent_name << "')");
    if (relative)
    {
        child->segment = KDL::Segment(child->segment.getName(), child->segment.getJoint(), pose, child->segment.getInertia());
    }
    else
    {
        child->segment = KDL::Segment(child->segment.getName(), child->segment.getJoint(), parent->frame.Inverse() * child->frame * pose, child->segment.getInertia());
    }

    // Iterate over Parent's children to find child and then remove it
    for (auto it = child->parent.lock()->children.begin(); it != child->parent.lock()->children.end();)
    {
        std::shared_ptr<KinematicElement> childOfParent = it->lock();
        if (childOfParent == child)
        {
            child->parent.lock()->children.erase(it);
        }
        else
        {
            ++it;
        }
    }

    child->parent = parent;
    child->parent_name = parent->segment.getName();
    parent->children.push_back(child);
    child->UpdateClosestRobotLink();
    debug_scene_changed_ = true;
}

std::shared_ptr<KinematicElement> KinematicTree::AddEnvironmentElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, const std::vector<VisualElement>& visual, bool is_controlled)
{
    std::shared_ptr<KinematicElement> element = AddElement(name, transform, parent, shape, inertia, color, visual, is_controlled);
    environment_tree_.push_back(element);
    return element;
}

std::shared_ptr<KinematicElement> KinematicTree::AddElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent, const std::string& shape_resource_path, Eigen::Vector3d scale, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, const std::vector<VisualElement>& visual, bool is_controlled)
{
    std::string shape_path(shape_resource_path);
    if (shape_path.empty())
    {
        ThrowPretty("Shape path cannot be empty!");
    }
    // Exotica package path resolution
    else if (shape_path.substr(0, 1) == "{")
    {
        shape_path = "file://" + ParsePath(shape_path);
    }
    // ROS resource path resolution
    else if (shape_path.substr(0, 10) == "package://" || shape_path.substr(0, 8) == "file:///")
    {
    }
    else
    {
        ThrowPretty("Path cannot be resolved.");
    }

    shapes::ShapePtr shape = shapes::ShapePtr(shapes::createMeshFromResource(shape_path, scale));
    std::shared_ptr<KinematicElement> element = AddElement(name, transform, parent, shape, inertia, color, visual, is_controlled);
    element->shape_resource_path = shape_path;
    element->scale = scale;
    return element;
}

std::shared_ptr<KinematicElement> KinematicTree::AddElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, const std::vector<VisualElement>& visual, bool is_controlled)
{
    std::shared_ptr<KinematicElement> parent_element;
    if (parent == "")
    {
        parent_element = root_;
    }
    else
    {
        bool found = false;
        for (const auto& element : tree_)
        {
            if (element.lock()->segment.getName() == parent)
            {
                parent_element = element.lock();
                found = true;
                break;
            }
        }
        if (!found) ThrowPretty("Can't find parent link named '" << parent << "'!");
    }
    KDL::Frame transform_kdl;
    tf::transformEigenToKDL(transform, transform_kdl);
    std::shared_ptr<KinematicElement> new_element = std::make_shared<KinematicElement>(tree_.size(), parent_element, KDL::Segment(name, KDL::Joint(KDL::Joint::None), transform_kdl, inertia));
    if (shape)
    {
        new_element->shape = shape;
        collision_tree_map_[new_element->segment.getName()] = new_element;

        // Set color if set. If all zeros, default to preset (grey).
        if (color != Eigen::Vector4d::Zero()) new_element->color = color;
    }
    new_element->parent_name = parent;
    new_element->is_controlled = is_controlled;
    tree_.push_back(new_element);
    parent_element->children.push_back(new_element);
    new_element->UpdateClosestRobotLink();
    tree_map_[name] = new_element;
    new_element->visual = visual;
    debug_scene_changed_ = true;
    return new_element;
}

void KinematicTree::AddElementFromSegmentMapIterator(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent)
{
    std::shared_ptr<KinematicElement> new_element = std::make_shared<KinematicElement>(model_tree_.size(), parent, segment->second.segment);
    model_tree_.push_back(new_element);
    if (parent) parent->children.push_back(new_element);
    for (KDL::SegmentMap::const_iterator child : segment->second.children)
    {
        AddElementFromSegmentMapIterator(child, new_element);
    }
}

int KinematicTree::IsControlled(std::shared_ptr<KinematicElement> joint)
{
    for (int i = 0; i < controlled_joints_names_.size(); ++i)
    {
        if (controlled_joints_names_[i] == joint->segment.getJoint().getName()) return i;
    }
    return -1;
}

int KinematicTree::IsControlledLink(const std::string& link_name)
{
    try
    {
        auto element = tree_map_[link_name].lock();

        if (element && element->is_controlled)
        {
            return element->control_id;
        }

        while (element)
        {
            element = element->parent.lock();

            if (element && element->is_controlled)
            {
                return element->control_id;
            }
        }
    }
    catch (const std::out_of_range& e)
    {
        return -1;
    }
    return -1;
}

std::shared_ptr<KinematicResponse> KinematicTree::RequestFrames(const KinematicsRequest& request)
{
    flags_ = request.flags;
    if (flags_ & KIN_H) flags_ = flags_ | KIN_J;
    solution_.reset(new KinematicResponse(flags_, request.frames.size(), num_controlled_joints_));

    state_size_ = num_controlled_joints_;

    for (int i = 0; i < request.frames.size(); ++i)
    {
        if (request.frames[i].frame_A_link_name == "")
            solution_->frame[i].frame_A = root_;
        else
            try
            {
                solution_->frame[i].frame_A = tree_map_.at(request.frames[i].frame_A_link_name);
            }
            catch (const std::out_of_range& e)
            {
                ThrowPretty("No frame_A link exists named '" << request.frames[i].frame_A_link_name << "'");
            }
        if (request.frames[i].frame_B_link_name == "")
            solution_->frame[i].frame_B = root_;
        else
            try
            {
                solution_->frame[i].frame_B = tree_map_.at(request.frames[i].frame_B_link_name);
            }
            catch (const std::out_of_range& e)
            {
                ThrowPretty("No frame_B link exists named '" << request.frames[i].frame_B_link_name << "'");
            }

        solution_->frame[i].frame_A_offset = request.frames[i].frame_A_offset;
        solution_->frame[i].frame_B_offset = request.frames[i].frame_B_offset;
    }

    if (debug)
    {
        for (KinematicFrame& frame : solution_->frame)
        {
            HIGHLIGHT(frame.frame_B.lock()->segment.getName() << " " << (frame.frame_B_offset == KDL::Frame::Identity() ? "" : ToString(frame.frame_B_offset)) << " -> " << frame.frame_A.lock()->segment.getName() << " " << (frame.frame_A_offset == KDL::Frame::Identity() ? "" : ToString(frame.frame_A_offset)));
        }
    }

    debug_frames_.resize(solution_->frame.size() * 2);

    return solution_;
}

void KinematicTree::Update(Eigen::VectorXdRefConst x)
{
    if (x.size() != state_size_) ThrowPretty("Wrong state vector size! Got " << x.size() << " expected " << state_size_);

    for (int i = 0; i < num_controlled_joints_; ++i)
        tree_state_(controlled_joints_[i].lock()->id) = x(i);

    // Store the updated state in the KinematicResponse (solution_)
    solution_->x = x;

    UpdateTree();
    UpdateFK();
    if (flags_ & KIN_J) UpdateJ();
    if (flags_ & KIN_J && flags_ & KIN_H) UpdateH();
    if (debug) PublishFrames();
}

void KinematicTree::UpdateTree()
{
    std::queue<std::shared_ptr<KinematicElement>> elements;
    elements.push(root_);
    root_->RemoveExpiredChildren();
    while (elements.size() > 0)
    {
        auto element = elements.front();
        elements.pop();
        // Elements with id > -1 have parent links.
        // ID=-1 is the global world reference frame.
        if (element->id > -1)
        {
            if (element->segment.getJoint().getType() != KDL::Joint::JointType::None)
            {
                element->frame = element->parent.lock()->frame * element->GetPose(tree_state_(element->id));
            }
            else
            {
                element->frame = element->parent.lock()->frame * element->GetPose();
            }
        }
        // Root of tree.
        else
        {
            // NB: We could simply set KDL::Frame() here, however, to support
            // trajectories for the base joint, we return GetPose();
            element->frame = element->GetPose();
        }
        element->RemoveExpiredChildren();
        for (std::weak_ptr<KinematicElement> child : element->children)
        {
            elements.push(child.lock());
        }
    }
}

void KinematicTree::PublishFrames(const std::string& tf_prefix)
{
    if (Server::IsRos())
    {
        const ros::Time timestamp = ros::Time::now();

        // Step 1: Publish frames for every element in the tree.
        {
            int i = 0;
            for (std::weak_ptr<KinematicElement> element : tree_)
            {
                tf::Transform T;
                tf::transformKDLToTF(element.lock()->frame, T);
                if (i > 0) debug_tree_[i - 1] = tf::StampedTransform(T, timestamp, tf::resolve(tf_prefix, GetRootFrameName()), tf::resolve(tf_prefix, element.lock()->segment.getName()));
                ++i;
            }
            Server::SendTransform(debug_tree_);
            i = 0;
            for (KinematicFrame& frame : solution_->frame)
            {
                tf::Transform T;
                tf::transformKDLToTF(frame.temp_B, T);
                debug_frames_[i * 2] = tf::StampedTransform(T, timestamp, tf::resolve(tf_prefix, GetRootFrameName()), tf::resolve(tf_prefix, "Frame" + std::to_string(i) + "B" + frame.frame_B.lock()->segment.getName()));
                tf::transformKDLToTF(frame.temp_AB, T);
                debug_frames_[i * 2 + 1] = tf::StampedTransform(T, timestamp, tf::resolve(tf_prefix, "Frame" + std::to_string(i) + "B" + frame.frame_B.lock()->segment.getName()), tf::resolve(tf_prefix, "Frame" + std::to_string(i) + "A" + frame.frame_A.lock()->segment.getName()));
                ++i;
            }
            Server::SendTransform(debug_frames_);
        }

        // Step 2: Publish visualisation markers for non-robot-model elements in the tree.
        if (debug_scene_changed_)
        {
            debug_scene_changed_ = false;
            marker_array_msg_.markers.clear();
            for (int i = 0; i < tree_.size(); ++i)
            {
                // Mesh from path
                if (tree_[i].lock()->shape_resource_path != "")
                {
                    visualization_msgs::Marker mrk;
                    mrk.action = visualization_msgs::Marker::ADD;
                    mrk.frame_locked = true;
                    mrk.id = i;
                    mrk.ns = "CollisionObjects";
                    mrk.color = GetColor(tree_[i].lock()->color);
                    mrk.header.frame_id = tf_prefix + "/" + tree_[i].lock()->segment.getName();
                    mrk.pose.orientation.w = 1.0;
                    mrk.type = visualization_msgs::Marker::MESH_RESOURCE;
                    mrk.mesh_resource = tree_[i].lock()->shape_resource_path;
                    mrk.mesh_use_embedded_materials = true;
                    mrk.scale.x = tree_[i].lock()->scale(0);
                    mrk.scale.y = tree_[i].lock()->scale(1);
                    mrk.scale.z = tree_[i].lock()->scale(2);
                    marker_array_msg_.markers.push_back(mrk);
                }
                // Non-robot collision objects
                else if (tree_[i].lock()->shape && (!tree_[i].lock()->closest_robot_link.lock() || !tree_[i].lock()->closest_robot_link.lock()->is_robot_link))
                {
                    if (tree_[i].lock()->shape->type != shapes::ShapeType::OCTREE)
                    {
                        visualization_msgs::Marker mrk;
                        shapes::constructMarkerFromShape(tree_[i].lock()->shape.get(), mrk);
                        mrk.action = visualization_msgs::Marker::ADD;
                        mrk.frame_locked = true;
                        mrk.id = i;
                        mrk.ns = "CollisionObjects";
                        mrk.color = GetColor(tree_[i].lock()->color);
                        mrk.header.frame_id = tf_prefix + "/" + tree_[i].lock()->segment.getName();
                        mrk.pose.orientation.w = 1.0;
                        marker_array_msg_.markers.push_back(mrk);
                    }
                    // Octree
                    else
                    {
                        // OcTree needs separate handling as it's not supported in constructMarkerFromShape
                        // NB: This only supports a single OctoMap in the KinematicTree as we only have one publisher!
                        octomap::OcTree my_octomap = *std::static_pointer_cast<const shapes::OcTree>(tree_[i].lock()->shape)->octree.get();
                        octomap_msgs::Octomap octomap_msg;
                        octomap_msgs::binaryMapToMsg(my_octomap, octomap_msg);
                        octomap_msg.header.frame_id = tf_prefix + "/" + tree_[i].lock()->segment.getName();
                        octomap_pub_.publish(octomap_msg);
                    }
                }
            }
            shapes_pub_.publish(marker_array_msg_);
        }
    }
}

KDL::Frame KinematicTree::FK(KinematicFrame& frame) const
{
    frame.temp_A = frame.frame_A.lock()->frame * frame.frame_A_offset;
    frame.temp_B = frame.frame_B.lock()->frame * frame.frame_B_offset;
    frame.temp_AB = frame.temp_B.Inverse() * frame.temp_A;
    return frame.temp_AB;
}

KDL::Frame KinematicTree::FK(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const
{
    if (!element_A) ThrowPretty("The pointer to KinematicElement A is dead.");
    KinematicFrame frame;
    frame.frame_A = element_A;
    frame.frame_B = (element_B == nullptr) ? root_ : element_B;
    frame.frame_A_offset = offset_a;
    frame.frame_B_offset = offset_b;
    return FK(frame);
}

KDL::Frame KinematicTree::FK(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const
{
    std::string name_a = element_A == "" ? root_->segment.getName() : element_A;
    std::string name_b = element_B == "" ? root_->segment.getName() : element_B;
    auto A = tree_map_.find(name_a);
    if (A == tree_map_.end()) ThrowPretty("Can't find link '" << name_a << "'!");
    auto B = tree_map_.find(name_b);
    if (B == tree_map_.end()) ThrowPretty("Can't find link '" << name_b << "'!");
    return FK(A->second.lock(), offset_a, B->second.lock(), offset_b);
}

void KinematicTree::UpdateFK()
{
    int i = 0;
    for (KinematicFrame& frame : solution_->frame)
    {
        solution_->Phi(i) = FK(frame);
        ++i;
    }
}

Eigen::MatrixXd KinematicTree::Jacobian(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const
{
    if (!element_A) ThrowPretty("The pointer to KinematicElement A is dead.");
    KinematicFrame frame;
    frame.frame_A = element_A;
    frame.frame_B = (element_B == nullptr) ? root_ : element_B;
    frame.frame_A_offset = offset_a;
    frame.frame_B_offset = offset_b;
    KDL::Jacobian ret(num_controlled_joints_);
    ComputeJ(frame, ret);
    return ret.data;
}

Eigen::MatrixXd KinematicTree::Jacobian(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const
{
    std::string name_a = element_A == "" ? root_->segment.getName() : element_A;
    std::string name_b = element_B == "" ? root_->segment.getName() : element_B;
    auto A = tree_map_.find(name_a);
    if (A == tree_map_.end()) ThrowPretty("Can't find link '" << name_a << "'!");
    auto B = tree_map_.find(name_b);
    if (B == tree_map_.end()) ThrowPretty("Can't find link '" << name_b << "'!");
    return Jacobian(A->second.lock(), offset_a, B->second.lock(), offset_b);
}

exotica::Hessian KinematicTree::Hessian(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const
{
    if (!element_A) ThrowPretty("The pointer to KinematicElement A is dead.");
    KinematicFrame frame;
    frame.frame_A = element_A;
    frame.frame_B = (element_B == nullptr) ? root_ : element_B;
    frame.frame_A_offset = offset_a;
    frame.frame_B_offset = offset_b;
    KDL::Jacobian J(num_controlled_joints_);
    ComputeJ(frame, J);
    exotica::Hessian hessian = exotica::Hessian::Constant(6, Eigen::MatrixXd::Zero(num_controlled_joints_, num_controlled_joints_));
    ComputeH(frame, J, hessian);
    return hessian;
}

exotica::Hessian KinematicTree::Hessian(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const
{
    std::string name_a = element_A == "" ? root_->segment.getName() : element_A;
    std::string name_b = element_B == "" ? root_->segment.getName() : element_B;
    auto A = tree_map_.find(name_a);
    if (A == tree_map_.end()) ThrowPretty("Can't find link '" << name_a << "'!");
    auto B = tree_map_.find(name_b);
    if (B == tree_map_.end()) ThrowPretty("Can't find link '" << name_b << "'!");
    return this->Hessian(A->second.lock(), offset_a, B->second.lock(), offset_b);
}

void KinematicTree::ComputeJ(KinematicFrame& frame, KDL::Jacobian& jacobian) const
{
    jacobian.data.setZero();
    (void)FK(frame);  // Create temporary offset frames
    std::shared_ptr<KinematicElement> it = frame.frame_A.lock();
    while (it != nullptr)
    {
        if (it->is_controlled)
        {
            KDL::Frame segment_reference;
            if (it->parent.lock() != nullptr) segment_reference = it->parent.lock()->frame;
            jacobian.setColumn(it->control_id, frame.temp_B.M.Inverse() * (segment_reference.M * it->segment.twist(tree_state_(it->id), 1.0)).RefPoint(frame.temp_A.p - it->frame.p));
        }
        it = it->parent.lock();
    }
    it = frame.frame_B.lock();
    while (it != nullptr)
    {
        if (it->is_controlled)
        {
            KDL::Frame segment_reference;
            if (it->parent.lock() != nullptr) segment_reference = it->parent.lock()->frame;
            jacobian.setColumn(it->control_id, jacobian.getColumn(it->control_id) - (frame.temp_B.M.Inverse() * (segment_reference.M * it->segment.twist(tree_state_(it->id), 1.0)).RefPoint(frame.temp_A.p - it->frame.p)));
        }
        it = it->parent.lock();
    }
}

void KinematicTree::ComputeH(KinematicFrame& frame, const KDL::Jacobian& jacobian, exotica::Hessian& hessian) const
{
    hessian.conservativeResize(6);
    for (int i = 0; i < 6; ++i)
    {
        hessian(i).resize(jacobian.columns(), jacobian.columns());
        hessian(i).setZero();
    }

    KDL::Twist axis;

    for (int i = 0; i < jacobian.columns(); ++i)
    {
        axis.rot = jacobian.getColumn(i).rot;
        for (int j = i; j < jacobian.columns(); ++j)
        {
            KDL::Twist Hij = axis * jacobian.getColumn(j);
            hessian(0)(i, j) = Hij[0];
            hessian(1)(i, j) = Hij[1];
            hessian(2)(i, j) = Hij[2];
            hessian(0)(j, i) = Hij[0];
            hessian(1)(j, i) = Hij[1];
            hessian(2)(j, i) = Hij[2];
            if (i != j)
            {
                hessian(3)(j, i) = Hij[3];
                hessian(4)(j, i) = Hij[4];
                hessian(5)(j, i) = Hij[5];
            }
        }
    }
}

void KinematicTree::UpdateJ()
{
    int i = 0;
    for (KinematicFrame& frame : solution_->frame)
    {
        ComputeJ(frame, solution_->jacobian(i));
        ++i;
    }
}

void KinematicTree::UpdateH()
{
    int i = 0;
    for (KinematicFrame& frame : solution_->frame)
    {
        ComputeH(frame, solution_->jacobian(i), solution_->hessian(i));
        ++i;
    }
}

exotica::BaseType KinematicTree::GetModelBaseType() const
{
    return model_base_type_;
}

exotica::BaseType KinematicTree::GetControlledBaseType() const
{
    return controlled_base_type_;
}

std::map<std::string, std::vector<double>> KinematicTree::GetUsedJointLimits() const
{
    std::map<std::string, std::vector<double>> limits;
    for (auto it : controlled_joints_)
    {
        limits[it.lock()->segment.getJoint().getName()] = it.lock()->joint_limits;
    }
    return limits;
}

robot_model::RobotModelPtr KinematicTree::GetRobotModel() const
{
    return model_;
}

Eigen::VectorXd KinematicTree::GetRandomControlledState()
{
    Eigen::VectorXd q_rand(num_controlled_joints_);
    for (unsigned int i = 0; i < num_controlled_joints_; ++i)
    {
        q_rand(i) = random_state_distributions_[i](generator_);
    }
    return q_rand;
}

void KinematicTree::SetJointLimitsLower(Eigen::VectorXdRefConst lower_in)
{
    if (lower_in.rows() == num_controlled_joints_)
    {
        for (unsigned int i = 0; i < num_controlled_joints_; ++i)
        {
            controlled_joints_[i].lock()->joint_limits[LIMIT_POSITION_LOWER] = lower_in(i);
        }
    }
    else
    {
        ThrowPretty("Got " << lower_in.rows() << " but " << num_controlled_joints_ << " expected.");
    }

    UpdateJointLimits();
}

void KinematicTree::SetJointLimitsUpper(Eigen::VectorXdRefConst upper_in)
{
    if (upper_in.rows() == num_controlled_joints_)
    {
        for (unsigned int i = 0; i < num_controlled_joints_; ++i)
        {
            controlled_joints_[i].lock()->joint_limits[LIMIT_POSITION_UPPER] = upper_in(i);
        }
    }
    else
    {
        ThrowPretty("Got " << upper_in.rows() << " but " << num_controlled_joints_ << " expected.");
    }

    UpdateJointLimits();
}

void KinematicTree::SetJointVelocityLimits(Eigen::VectorXdRefConst velocity_in)
{
    if (velocity_in.rows() == num_controlled_joints_)
    {
        for (unsigned int i = 0; i < num_controlled_joints_; ++i)
        {
            controlled_joints_[i].lock()->velocity_limit = velocity_in(i);
        }
    }
    else
    {
        ThrowPretty("Got " << velocity_in.rows() << " but " << num_controlled_joints_ << " expected.");
    }

    UpdateJointLimits();
}

void KinematicTree::SetJointAccelerationLimits(Eigen::VectorXdRefConst acceleration_in)
{
    if (acceleration_in.rows() == num_controlled_joints_)
    {
        for (unsigned int i = 0; i < num_controlled_joints_; ++i)
        {
            controlled_joints_[i].lock()->acceleration_limit = acceleration_in(i);
        }

        has_acceleration_limit_ = true;
    }
    else
    {
        ThrowPretty("Got " << acceleration_in.rows() << " but " << num_controlled_joints_ << " expected.");
    }

    UpdateJointLimits();
}

void KinematicTree::SetFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double>& lower, const std::vector<double>& upper)
{
    if (controlled_base_type_ != BaseType::FLOATING)
    {
        ThrowPretty("This is not a floating joint!");
    }
    if (lower.size() != 6 || upper.size() != 6)
    {
        ThrowPretty("Wrong limit data size!");
    }
    for (int i = 0; i < 6; ++i)
    {
        controlled_joints_[i].lock()->joint_limits = {lower[i], upper[i]};
    }

    UpdateJointLimits();
}

void KinematicTree::SetFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double>& lower, const std::vector<double>& upper, const std::vector<double>& velocity, const std::vector<double>& acceleration)
{
    if (controlled_base_type_ != BaseType::FLOATING)
    {
        ThrowPretty("This is not a floating joint!");
    }
    if (lower.size() != 6 || upper.size() != 6)
    {
        ThrowPretty("Wrong joint limit data size!");
    }
    if (velocity.size() != 6 && velocity.size() != 0)
    {
        ThrowPretty("Wrong velocity limit size!");
    }
    if (acceleration.size() != 6 && acceleration.size() != 0)
    {
        ThrowPretty("Wrong acceleration limit size!");
    }
    for (int i = 0; i < 6; ++i)
    {
        controlled_joints_[i].lock()->joint_limits = {lower[i], upper[i]};
        controlled_joints_[i].lock()->velocity_limit = {velocity.size() != 0 ? velocity[i] : inf};
        controlled_joints_[i].lock()->acceleration_limit = {acceleration.size() != 0 ? acceleration[i] : inf};
    }

    UpdateJointLimits();
}

void KinematicTree::SetPlanarBaseLimitsPosXYEulerZ(
    const std::vector<double>& lower, const std::vector<double>& upper)
{
    if (controlled_base_type_ != BaseType::PLANAR)
    {
        ThrowPretty("This is not a planar joint!");
    }
    if (lower.size() != 3 || upper.size() != 3)
    {
        ThrowPretty("Wrong limit data size!");
    }
    for (int i = 0; i < 3; ++i)
    {
        controlled_joints_[i].lock()->joint_limits = {lower[i], upper[i]};
    }

    UpdateJointLimits();
}

void KinematicTree::SetPlanarBaseLimitsPosXYEulerZ(
    const std::vector<double>& lower, const std::vector<double>& upper, const std::vector<double>& velocity, const std::vector<double>& acceleration)
{
    if (controlled_base_type_ != BaseType::PLANAR)
    {
        ThrowPretty("This is not a planar joint!");
    }
    if (lower.size() != 3 || upper.size() != 3)
    {
        ThrowPretty("Wrong joint limit data size!");
    }
    if (velocity.size() != 3 && velocity.size() != 0)
    {
        ThrowPretty("Wrong velocity limit size!");
    }
    if (acceleration.size() != 3 && acceleration.size() != 0)
    {
        ThrowPretty("Wrong acceleration limit size!");
    }
    for (int i = 0; i < 3; ++i)
    {
        controlled_joints_[i].lock()->joint_limits = {lower[i], upper[i]};
        controlled_joints_[i].lock()->velocity_limit = {velocity.size() != 0 ? velocity[i] : inf};
        controlled_joints_[i].lock()->acceleration_limit = {acceleration.size() != 0 ? acceleration[i] : inf};
    }

    UpdateJointLimits();
}

void KinematicTree::ResetJointLimits()
{
    std::vector<std::string> vars = model_->getVariableNames();
    for (int i = 0; i < vars.size(); ++i)
    {
        if (controlled_joints_map_.find(vars[i]) != controlled_joints_map_.end())
        {
            auto& ControlledJoint = controlled_joints_map_.at(vars[i]);
            int index = ControlledJoint.lock()->control_id;

            // moveit_core::RobotModel does not have effort limits
            // TODO: use urdf::Model instead

            // Check for bounds, else set limits to inf
            if (model_->getVariableBounds(vars[i]).position_bounded_)
            {
                controlled_joints_[index].lock()->joint_limits = {model_->getVariableBounds(vars[i]).min_position_, model_->getVariableBounds(vars[i]).max_position_};
            }
            else
            {
                controlled_joints_[index].lock()->joint_limits = {-inf, inf};
            }
            if (model_->getVariableBounds(vars[i]).velocity_bounded_)
            {
                controlled_joints_[index].lock()->velocity_limit = {model_->getVariableBounds(vars[i]).max_velocity_};
            }
            else
            {
                controlled_joints_[index].lock()->velocity_limit = {inf};
            }
            if (model_->getVariableBounds(vars[i]).acceleration_bounded_)
            {
                controlled_joints_[index].lock()->acceleration_limit = {model_->getVariableBounds(vars[i]).max_acceleration_};
            }
            else
            {
                controlled_joints_[index].lock()->acceleration_limit = {inf};
            }
        }
    }

    ///	Manually defined floating base limits
    ///	Should be done more systematically with robot model class
    if (controlled_base_type_ == BaseType::FLOATING)
    {
        controlled_joints_[0].lock()->joint_limits = {-inf, inf};
        controlled_joints_[1].lock()->joint_limits = {-inf, inf};
        controlled_joints_[2].lock()->joint_limits = {-inf, inf};
        controlled_joints_[3].lock()->joint_limits = {-pi, pi};
        controlled_joints_[4].lock()->joint_limits = {-pi, pi};
        controlled_joints_[5].lock()->joint_limits = {-pi, pi};
    }
    else if (controlled_base_type_ == BaseType::PLANAR)
    {
        controlled_joints_[0].lock()->joint_limits = {-inf, inf};
        controlled_joints_[1].lock()->joint_limits = {-inf, inf};
        controlled_joints_[2].lock()->joint_limits = {-pi, pi};
    }

    UpdateJointLimits();
}

void KinematicTree::UpdateJointLimits()
{
    joint_limits_.setZero();
    for (int i = 0; i < num_controlled_joints_; ++i)
    {
        joint_limits_(i, LIMIT_POSITION_LOWER) = controlled_joints_[i].lock()->joint_limits[LIMIT_POSITION_LOWER];
        joint_limits_(i, LIMIT_POSITION_UPPER) = controlled_joints_[i].lock()->joint_limits[LIMIT_POSITION_UPPER];
        velocity_limits_(i) = controlled_joints_[i].lock()->velocity_limit;
        acceleration_limits_(i) = controlled_joints_[i].lock()->acceleration_limit;
    }

    // Update random state distributions for generating random controlled states
    random_state_distributions_.clear();
    for (int i = 0; i < num_controlled_joints_; ++i)
    {
        random_state_distributions_.push_back(std::uniform_real_distribution<double>(joint_limits_(i, LIMIT_POSITION_LOWER), joint_limits_(i, LIMIT_POSITION_UPPER)));
        // TODO: Implement random velocities and accelerations
    }
}

const std::string& KinematicTree::GetRootFrameName() const
{
    return root_->segment.getName();
}

const std::string& KinematicTree::GetRootJointName() const
{
    return root_joint_name_;
}

Eigen::VectorXd KinematicTree::GetModelState() const
{
    Eigen::VectorXd ret(model_joints_names_.size());

    for (int i = 0; i < model_joints_names_.size(); ++i)
    {
        ret(i) = tree_state_(model_joints_map_.at(model_joints_names_[i]).lock()->id);
    }
    return ret;
}

std::map<std::string, double> KinematicTree::GetModelStateMap() const
{
    std::map<std::string, double> ret;
    for (const std::string& joint_name : model_joints_names_)
    {
        ret[joint_name] = tree_state_(model_joints_map_.at(joint_name).lock()->id);
    }
    return ret;
}

std::vector<std::string> KinematicTree::GetKinematicChain(const std::string& begin, const std::string& end) const
{
    // check existence of requested links
    for (const std::string& l : {begin, end})
    {
        if (!tree_map_.count(l))
        {
            ThrowPretty("Link '" + l + "' does not exist.");
        }
    }

    // get chain in reverse order, end...begin
    std::vector<std::string> chain;
    for (std::weak_ptr<KinematicElement> l = tree_map_.at(end);
         l.lock()->segment.getName() != begin;
         l = l.lock()->parent, chain.push_back(l.lock()->segment.getJoint().getName()))
    {
        if (l.lock()->parent.lock() == nullptr)
        {
            ThrowPretty("There is no connection between '" + begin + "' and '" + end + "'!");
        }
    }

    // return vector in order, begin...end
    std::reverse(chain.begin(), chain.end());
    return chain;
}

std::vector<std::string> KinematicTree::GetKinematicChainLinks(const std::string& begin, const std::string& end) const
{
    // check existence of requested links
    for (const std::string& l : {begin, end})
    {
        if (!tree_map_.count(l))
        {
            ThrowPretty("Link '" + l + "' does not exist.");
        }
    }

    // get chain in reverse order, end...begin
    std::vector<std::string> chain;
    for (std::shared_ptr<const KinematicElement> l = tree_map_.at(end).lock(); l->segment.getName() != begin; l = l->parent.lock())
    {
        chain.push_back(l->segment.getName());
        if (l->parent.lock() == nullptr)
        {
            ThrowPretty("There is no connection between '" + begin + "' and '" + end + "'!");
        }
    }

    // return vector in order, begin...end
    std::reverse(chain.begin(), chain.end());
    return chain;
}

void KinematicTree::SetModelState(Eigen::VectorXdRefConst x)
{
    // Work-around in case someone passed a vector of size num_controlled_joints_
    if (x.rows() == num_controlled_joints_)
    {
        Update(x);
        return;
    }

    if (x.rows() != model_joints_names_.size()) ThrowPretty("Model state vector has wrong size, expected " << model_joints_names_.size() << " got " << x.rows());
    for (int i = 0; i < model_joints_names_.size(); ++i)
    {
        tree_state_(model_joints_map_.at(model_joints_names_[i]).lock()->id) = x(i);
    }
    UpdateTree();
    UpdateFK();
    if (flags_ & KIN_J) UpdateJ();
    if (debug) PublishFrames();
}

void KinematicTree::SetModelState(const std::map<std::string, double>& x)
{
    for (auto& joint : x)
    {
        try
        {
            tree_state_(model_joints_map_.at(joint.first).lock()->id) = joint.second;
        }
        catch (const std::out_of_range& e)
        {
            WARNING("Robot model does not contain joint '" << joint.first << "' - ignoring.");
        }
    }
    UpdateTree();
    UpdateFK();
    if (flags_ & KIN_J) UpdateJ();
    if (debug) PublishFrames();
}

Eigen::VectorXd KinematicTree::GetControlledState() const
{
    Eigen::VectorXd x(num_controlled_joints_);
    for (int i = 0; i < controlled_joints_.size(); ++i)
    {
        x(i) = tree_state_(controlled_joints_[i].lock()->id);
    }
    return x;
}

bool KinematicTree::HasModelLink(const std::string& link) const
{
    return std::find(std::begin(model_link_names_), std::end(model_link_names_), link) != std::end(model_link_names_);
}

Eigen::VectorXd KinematicTree::GetControlledLinkMass() const
{
    Eigen::VectorXd x(num_controlled_joints_);
    for (int i = 0; i < controlled_joints_.size(); ++i)
    {
        x(i) = controlled_joints_[i].lock()->segment.getInertia().getMass();
    }
    return x;
}

std::map<std::string, shapes::ShapeType> KinematicTree::GetCollisionObjectTypes() const
{
    std::map<std::string, shapes::ShapeType> ret;
    for (const auto& element : collision_tree_map_)
    {
        ret[element.second.lock()->segment.getName()] = element.second.lock()->shape->type;
    }
    return ret;
}

bool KinematicTree::DoesLinkWithNameExist(std::string name) const
{
    // Check whether it exists in TreeMap, which should encompass both EnvironmentTree and model_tree_
    return tree_map_.find(name) != tree_map_.end();
}

std::shared_ptr<KinematicElement> KinematicTree::FindKinematicElementByName(const std::string& frame_name)
{
    auto it = tree_map_.find(frame_name);
    if (it == tree_map_.end()) ThrowPretty("KinematicElement does not exist:" << frame_name);

    return it->second.lock();
}
}  // namespace exotica
