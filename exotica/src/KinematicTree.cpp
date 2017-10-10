/*
 *      Author: Michael Camilleri
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

#include "exotica/KinematicTree.h"
#include <moveit/robot_model/robot_model.h>
#include <algorithm>
#include <iostream>
#include <queue>

#include <eigen_conversions/eigen_kdl.h>
#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl/frames_io.hpp>

namespace exotica
{
KinematicResponse::KinematicResponse() : Flags(KIN_FK)
{
}

KinematicResponse::KinematicResponse(KinematicRequestFlags flags, int size, int n)
{
    Flags = flags;
    Frame.resize(size);
    Phi.resize(size);
    if (Flags & KIN_FK_VEL) PhiDot.resize(size);
    KDL::Jacobian Jzero(n);
    Jzero.data.setZero();
    if (Flags & KIN_J) J = ArrayJacobian::Constant(size, Jzero);
    if (Flags & KIN_J_DOT) JDot = ArrayJacobian::Constant(size, Jzero);
}

KinematicsRequest::KinematicsRequest() : Flags(KIN_FK)
{
}

KinematicFrameRequest::KinematicFrameRequest()
{
}

KinematicFrameRequest::KinematicFrameRequest(std::string frameALinkName, KDL::Frame frameAOffset, std::string frameBLinkName, KDL::Frame frameBOffset) : FrameALinkName(frameALinkName), FrameAOffset(frameAOffset), FrameBLinkName(frameBLinkName), FrameBOffset(frameBOffset)
{
}

KinematicSolution::KinematicSolution() : Start(-1), Length(-1), Phi(nullptr, 0), PhiDot(nullptr, 0), J(nullptr, 0), JDot(nullptr, 0)
{
}

KinematicSolution::KinematicSolution(int start, int length) : Start(start), Length(length), Phi(nullptr, 0), PhiDot(nullptr, 0), J(nullptr, 0), JDot(nullptr, 0)
{
}

void KinematicSolution::Create(std::shared_ptr<KinematicResponse> solution)
{
    if (Start < 0 || Length < 0) throw_pretty("Kinematic solution was not initialized!");
    new (&Phi) Eigen::Map<ArrayFrame>(solution->Phi.data() + Start, Length);
    if (solution->Flags & KIN_FK_VEL) new (&PhiDot) Eigen::Map<ArrayTwist>(solution->PhiDot.data() + Start, Length);
    if (solution->Flags & KIN_J) new (&J) Eigen::Map<ArrayJacobian>(solution->J.data() + Start, Length);
    if (solution->Flags & KIN_J_DOT) new (&JDot) Eigen::Map<ArrayJacobian>(solution->JDot.data() + Start, Length);
}

int KinematicTree::getNumControlledJoints()
{
    return NumControlledJoints;
}

int KinematicTree::getNumModelJoints()
{
    return NumJoints;
}

KinematicTree::KinematicTree() : StateSize(-1), Debug(false)
{
}

void KinematicTree::Instantiate(std::string JointGroup, robot_model::RobotModelPtr model, const std::string& name)
{
    if (!model) throw_pretty("No robot model provided!");
    robot_model::JointModelGroup* group = model->getJointModelGroup(JointGroup);
    if (!group) throw_pretty("Joint group '" << JointGroup << "' not defined in the robot model!");
    ControlledJointsNames = group->getVariableNames();
    ModelJointsNames = model->getVariableNames();
    name_ = name;

    Model = model;
    KDL::Tree RobotKinematics;
    if (kdl_parser::treeFromUrdfModel(*Model->getURDF(), RobotKinematics))
    {
        BuildTree(RobotKinematics);
    }
    else
    {
        throw_pretty("Can't load URDF model!");
    }

    if (Server::isRos())
    {
        shapes_pub_ = Server::advertise<visualization_msgs::MarkerArray>(name_ + (name_ == "" ? "" : "/") + "CollisionShapes", 100, true);
        debugSceneChanged = true;
    }
}

void KinematicTree::BuildTree(const KDL::Tree& RobotKinematics)
{
    ModelTree.clear();
    Tree.clear();
    TreeMap.clear();
    ModelJointsMap.clear();
    ModelLinkNames.clear();
    ControlledLinkNames.clear();

    // Handle the root joint
    const robot_model::JointModel* RootJoint = Model->getRootJoint();
    std::string WorldFrameName;
    for (const srdf::Model::VirtualJoint& s : Model->getSRDF()->getVirtualJoints())
    {
        if (s.name_ == RootJoint->getName())
        {
            WorldFrameName = s.parent_frame_;
        }
    }
    if (WorldFrameName == "") throw_pretty("Can't initialize root joint!");

    // Extract Root Inertial
    double RootMass = 0.0;
    KDL::Vector RootCoG = KDL::Vector::Zero();
    auto& UrdfRootInertial = Model->getURDF()->getRoot()->inertial;
    if (UrdfRootInertial)
    {
        RootMass = UrdfRootInertial->mass;
        RootCoG = KDL::Vector(UrdfRootInertial->origin.position.x,
                              UrdfRootInertial->origin.position.y,
                              UrdfRootInertial->origin.position.z);
        if (Debug)
            HIGHLIGHT_NAMED("Root Inertial", "Mass: " << RootMass
                                                      << " kg - CoG: " << RootCoG);
    }
    // TODO: Note, this does not set the rotational inertia component, i.e. the
    // inertial matrix would be wrong
    KDL::RigidBodyInertia RootInertial(RootMass, RootCoG);

    // Add general world_frame joint
    ModelTree.push_back(std::make_shared<KinematicElement>(ModelTree.size(), nullptr, KDL::Segment(WorldFrameName, KDL::Joint(RootJoint->getName(), KDL::Joint::None))));
    if (RootJoint->getType() == robot_model::JointModel::FIXED)
    {
        ModelBaseType = BASE_TYPE::FIXED;
    }
    else if (RootJoint->getType() == robot_model::JointModel::FLOATING)
    {
        ModelBaseType = BASE_TYPE::FLOATING;
        ModelTree.resize(7);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::TransZ, KDL::Joint::RotZ, KDL::Joint::RotY, KDL::Joint::RotX};
        std::vector<std::string> floatingBaseVariableNames = {
            RootJoint->getName() + "/trans_x",
            RootJoint->getName() + "/trans_y",
            RootJoint->getName() + "/trans_z",
            RootJoint->getName() + "/rot_z",
            RootJoint->getName() + "/rot_y",
            RootJoint->getName() + "/rot_x"};
        for (int i = 0; i < 6; i++)
        {
            ModelTree[i + 1] = std::make_shared<KinematicElement>(i, ModelTree[i], KDL::Segment(floatingBaseVariableNames[i], KDL::Joint(floatingBaseVariableNames[i], types[i])));
            ModelTree[i]->Children.push_back(ModelTree[i + 1]);
        }

        // The floating base rotation is defined as xyzw quaternion in the robot
        // model, but we are following a RPY-fixed axis (YPR rotating axis)
        // virtual joint convention in exotica - thus delete the rot_w from the
        // list of joint names
        auto RotW = std::find(ControlledJointsNames.begin(), ControlledJointsNames.end(), RootJoint->getVariableNames()[6]);
        if (RotW != ControlledJointsNames.end()) ControlledJointsNames.erase(RotW);
        RotW = std::find(ModelJointsNames.begin(), ModelJointsNames.end(), RootJoint->getVariableNames()[6]);
        if (RotW != ModelJointsNames.end()) ModelJointsNames.erase(RotW);
    }
    else if (RootJoint->getType() == robot_model::JointModel::PLANAR)
    {
        ModelBaseType = BASE_TYPE::PLANAR;
        ModelTree.resize(4);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY,
                                         KDL::Joint::RotZ};
        for (int i = 0; i < 3; i++)
        {
            ModelTree[i + 1] = std::make_shared<KinematicElement>(
                i, ModelTree[i],
                KDL::Segment(
                    RootJoint->getVariableNames()[i],
                    KDL::Joint(RootJoint->getVariableNames()[i], types[i])));
            ModelTree[i]->Children.push_back(ModelTree[i + 1]);
        }
    }
    else
    {
        throw_pretty("Unsupported root joint type: " << RootJoint->getTypeName());
    }

    AddElement(RobotKinematics.getRootSegment(), *(ModelTree.end() - 1));

    // Set root inertial
    if (RootJoint->getType() == robot_model::JointModel::FIXED)
    {
        ModelTree[2]->Segment.setInertia(RootInertial);
    }
    else if (RootJoint->getType() == robot_model::JointModel::FLOATING)
    {
        ModelTree[7]->Segment.setInertia(RootInertial);
    }
    else if (RootJoint->getType() == robot_model::JointModel::PLANAR)
    {
        ModelTree[4]->Segment.setInertia(RootInertial);
    }

    for (auto element : ModelTree) Tree.push_back(element);

    UpdateModel();
    TreeState.setZero();

    if (Debug)
    {
        for (int i = 0; i < Tree.size() - 1; i++)
            HIGHLIGHT_NAMED(
                "Tree", "Joint: " << Tree[i].lock()->Segment.getJoint().getName() << " - Link: " << Tree[i].lock()->Segment.getName()
                                  << ", mass: " << Tree[i].lock()->Segment.getInertia().getMass()
                                  << ", CoM: " << Tree[i].lock()->Segment.getInertia().getCOG());
    }

    NumJoints = ModelJointsNames.size();
    NumControlledJoints = ControlledJointsNames.size();
    if (NumControlledJoints < 1) throw_pretty("No update joints specified!");
    ControlledJoints.resize(NumControlledJoints);
    for (std::shared_ptr<KinematicElement> Joint : ModelTree)
    {
        Joint->isRobotLink = true;
        Joint->ControlId = IsControlled(Joint);
        Joint->IsControlled = Joint->ControlId >= 0;
        ModelJointsMap[Joint->Segment.getJoint().getName()] = Joint;
        ModelLinkNames.push_back(Joint->Segment.getName());
        if (Joint->IsControlled)
        {
            ControlledJoints[Joint->ControlId] = Joint;
            ControlledJointsMap[Joint->Segment.getJoint().getName()] = Joint;
            ControlledLinkNames.push_back(Joint->Segment.getName());

            // The ModelBaseType defined above refers to the base type of the
            // overall robot model - not of the set of controlled joints. E.g. a
            // floating-base robot can have a scene defined where the
            // floating-base virtual joint is _not_ part of the planning
            // group/scene. Thus we need to establish the BaseType of the joint
            // group, the ControlledBaseType - if a controlled joint corresponds
            // to a floating base joint, the ControlledBaseType is the same as the
            // ModelBaseType.
            if (Joint->Segment.getJoint().getName().find(
                    RootJoint->getName()) != std::string::npos)
                ControlledBaseType = ModelBaseType;
        }
    }
    ModelTree[0]->isRobotLink = false;

    setJointLimits();
}

void KinematicTree::UpdateModel()
{
    Root = Tree[0].lock();
    TreeState.conservativeResize(Tree.size());
    for (std::weak_ptr<KinematicElement> Joint : Tree)
    {
        TreeMap[Joint.lock()->Segment.getName()] = Joint.lock();
    }
    debugTree.resize(Tree.size() - 1);
    UpdateTree();
    debugSceneChanged = true;
}

void KinematicTree::resetModel()
{
    CollisionTreeMap.clear();
    TreeMap.clear();
    EnvironmentTree.clear();
    Tree.resize(ModelTree.size());
    UpdateModel();
    debugSceneChanged = true;

    // Remove all CollisionShapes
    if (Server::isRos())
    {
        visualization_msgs::MarkerArray msg;
        visualization_msgs::Marker mrk;
        mrk.action = 3;  // visualization_msgs::Marker::DELETEALL; // NB: enum only defined in ROS-J and newer, functionality still there
        msg.markers.push_back(mrk);
        shapes_pub_.publish(msg);
    }
}

void KinematicTree::changeParent(const std::string& name, const std::string& parent_name, const KDL::Frame& pose, bool relative)
{
    if (TreeMap.find(name) == TreeMap.end()) throw_pretty("Attempting to attach unknown frame '" << name << "'!");
    std::shared_ptr<KinematicElement> child = TreeMap.find(name)->second.lock();
    if (child->Id < ModelTree.size()) throw_pretty("Can't re-attach robot link '" << name << "'!");
    if (child->Shape) throw_pretty("Can't re-attach collision shape without reattaching the object! ('" << name << "')");
    std::shared_ptr<KinematicElement> parent;
    if (parent_name == "")
    {
        if (TreeMap.find(Root->Segment.getName()) == TreeMap.end()) throw_pretty("Attempting to attach to unknown frame '" << Root->Segment.getName() << "'!");
        parent = TreeMap.find(Root->Segment.getName())->second.lock();
    }
    else
    {
        if (TreeMap.find(parent_name) == TreeMap.end()) throw_pretty("Attempting to attach to unknown frame '" << parent_name << "'!");
        parent = TreeMap.find(parent_name)->second.lock();
    }
    if (parent->Shape) throw_pretty("Can't attach object to a collision shape object! ('" << parent_name << "')");
    if (relative)
    {
        child->Segment = KDL::Segment(child->Segment.getName(), child->Segment.getJoint(), pose, child->Segment.getInertia());
    }
    else
    {
        child->Segment = KDL::Segment(child->Segment.getName(), child->Segment.getJoint(), parent->Frame.Inverse() * child->Frame * pose, child->Segment.getInertia());
    }

    // Iterate over Parent's children to find child and then remove it
    for (auto it = child->Parent.lock()->Children.begin(); it != child->Parent.lock()->Children.end();)
    {
        std::shared_ptr<KinematicElement> childOfParent = it->lock();
        if (childOfParent == child)
        {
            child->Parent.lock()->Children.erase(it);
        }
        else
        {
            ++it;
        }
    }

    child->Parent = parent;
    child->ParentName = parent->Segment.getName();
    parent->Children.push_back(child);
    child->updateClosestRobotLink();
    debugSceneChanged = true;
}

void KinematicTree::AddEnvironmentElement(const std::string& name, Eigen::Affine3d& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, bool isControlled)
{
    std::shared_ptr<KinematicElement> element = AddElement(name, transform, parent, shape, inertia, color, isControlled);
    EnvironmentTree.push_back(element);
}

std::shared_ptr<KinematicElement> KinematicTree::AddElement(const std::string& name, Eigen::Affine3d& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, const Eigen::Vector4d& color, bool isControlled)
{
    std::shared_ptr<KinematicElement> parent_element;
    if (parent == "")
    {
        parent_element = Root;
    }
    else
    {
        bool found = false;
        for (const auto& element : Tree)
        {
            if (element.lock()->Segment.getName() == parent)
            {
                parent_element = element.lock();
                found = true;
                break;
            }
        }
        if (!found) throw_pretty("Can't find parent link named '" << parent << "'!");
    }
    KDL::Frame transformKDL;
    tf::transformEigenToKDL(transform, transformKDL);
    std::shared_ptr<KinematicElement> NewElement = std::make_shared<KinematicElement>(Tree.size(), parent_element, KDL::Segment(name, KDL::Joint(KDL::Joint::None), transformKDL, inertia));
    if (shape)
    {
        NewElement->Shape = shape;
        CollisionTreeMap[NewElement->Segment.getName()] = NewElement;

        // Set color if set. If all zeros, default to preset (grey).
        if (color != Eigen::Vector4d::Zero()) NewElement->Color = color;
    }
    NewElement->ParentName = parent;
    NewElement->IsControlled = isControlled;
    Tree.push_back(NewElement);
    parent_element->Children.push_back(NewElement);
    NewElement->updateClosestRobotLink();
    TreeMap[name] = NewElement;
    debugSceneChanged = true;
    return NewElement;
}

void KinematicTree::AddElement(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent)
{
    std::shared_ptr<KinematicElement> NewElement = std::make_shared<KinematicElement>(ModelTree.size(), parent, segment->second.segment);
    ModelTree.push_back(NewElement);
    if (parent) parent->Children.push_back(NewElement);
    for (KDL::SegmentMap::const_iterator child : segment->second.children)
    {
        AddElement(child, NewElement);
    }
}

int KinematicTree::IsControlled(std::shared_ptr<KinematicElement> Joint)
{
    for (int i = 0; i < ControlledJointsNames.size(); i++)
    {
        if (ControlledJointsNames[i] == Joint->Segment.getJoint().getName()) return i;
    }
    return -1;
}

std::shared_ptr<KinematicResponse> KinematicTree::RequestFrames(const KinematicsRequest& request)
{
    Flags = request.Flags;
    if (Flags & KIN_J_DOT) Flags = Flags | KIN_J;
    Solution.reset(new KinematicResponse(Flags, request.Frames.size(), NumControlledJoints));

    StateSize = NumControlledJoints;
    if (((Flags & KIN_FK_VEL) || (Flags & KIN_J_DOT))) StateSize = NumControlledJoints;

    for (int i = 0; i < request.Frames.size(); i++)
    {
        if (request.Frames[i].FrameALinkName == "")
            Solution->Frame[i].FrameA = Root;
        else
            try
            {
                Solution->Frame[i].FrameA = TreeMap.at(request.Frames[i].FrameALinkName);
            }
            catch (const std::out_of_range& e)
            {
                throw_pretty("No FrameA link exists named '" << request.Frames[i].FrameALinkName << "'");
            }
        if (request.Frames[i].FrameBLinkName == "")
            Solution->Frame[i].FrameB = Root;
        else
            try
            {
                Solution->Frame[i].FrameB = TreeMap.at(request.Frames[i].FrameBLinkName);
            }
            catch (const std::out_of_range& e)
            {
                throw_pretty("No FrameB link exists named '" << request.Frames[i].FrameBLinkName << "'");
            }

        Solution->Frame[i].FrameAOffset = request.Frames[i].FrameAOffset;
        Solution->Frame[i].FrameBOffset = request.Frames[i].FrameBOffset;
    }

    if (Debug)
    {
        for (KinematicFrame& frame : Solution->Frame)
        {
            HIGHLIGHT(frame.FrameB.lock()->Segment.getName() << " " << (frame.FrameBOffset == KDL::Frame::Identity() ? "" : toString(frame.FrameBOffset)) << " -> " << frame.FrameA.lock()->Segment.getName() << " " << (frame.FrameAOffset == KDL::Frame::Identity() ? "" : toString(frame.FrameAOffset)));
        }
    }

    debugFrames.resize(Solution->Frame.size() * 2);

    return Solution;
}

void KinematicTree::Update(Eigen::VectorXdRefConst x)
{
    if (x.rows() != StateSize) throw_pretty("Wrong state vector size! Got " << x.rows() << " expected " << StateSize);

    for (int i = 0; i < ControlledJoints.size(); i++)
        TreeState(ControlledJoints[i].lock()->Id) = x(i);

    UpdateTree();
    UpdateFK();
    if (Flags & KIN_J) UpdateJ();
    if (Flags & KIN_J && Flags & KIN_J_DOT) UpdateJdot();
    if (Debug) publishFrames();
}

void KinematicTree::UpdateTree()
{
    std::queue<std::shared_ptr<KinematicElement>> elements;
    elements.push(Root);
    Root->removeExpiredChildren();
    while (elements.size() > 0)
    {
        auto element = elements.front();
        elements.pop();
        if (element->Id > 0)
        {
            element->Frame = element->Parent.lock()->Frame * element->getPose(TreeState(element->Id));
        }
        else
        {
            element->Frame = element->getPose(TreeState(element->Id));
        }
        element->removeExpiredChildren();
        for (std::weak_ptr<KinematicElement> child : element->Children)
        {
            elements.push(child.lock());
        }
    }
}

void KinematicTree::publishFrames()
{
    if (Server::isRos())
    {
        int i = 0;
        for (std::weak_ptr<KinematicElement> element : Tree)
        {
            tf::Transform T;
            tf::transformKDLToTF(element.lock()->Frame, T);
            if (i > 0) debugTree[i - 1] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica", getRootFrameName()), tf::resolve("exotica", element.lock()->Segment.getName()));
            i++;
        }
        Server::sendTransform(debugTree);
        i = 0;
        for (KinematicFrame& frame : Solution->Frame)
        {
            tf::Transform T;
            tf::transformKDLToTF(frame.TempB, T);
            debugFrames[i * 2] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica", getRootFrameName()), tf::resolve("exotica", "Frame" + std::to_string(i) + "B" + frame.FrameB.lock()->Segment.getName()));
            tf::transformKDLToTF(frame.TempAB, T);
            debugFrames[i * 2 + 1] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica", "Frame" + std::to_string(i) + "B" + frame.FrameB.lock()->Segment.getName()), tf::resolve("exotica", "Frame" + std::to_string(i) + "A" + frame.FrameA.lock()->Segment.getName()));
            i++;
        }
        Server::sendTransform(debugFrames);
        if (debugSceneChanged)
        {
            debugSceneChanged = false;
            visualization_msgs::MarkerArray msg;
            for (int i = 0; i < Tree.size(); i++)
            {
                if (Tree[i].lock()->Shape && Tree[i].lock()->Parent.lock()->Id >= ModelTree.size())
                {
                    visualization_msgs::Marker mrk;
                    shapes::constructMarkerFromShape(Tree[i].lock()->Shape.get(), mrk);
                    mrk.action = visualization_msgs::Marker::ADD;
                    mrk.frame_locked = true;
                    mrk.id = i;
                    mrk.ns = "CollisionObjects";
                    mrk.color = getColor(Tree[i].lock()->Color);
                    mrk.header.frame_id = "exotica/" + Tree[i].lock()->Segment.getName();
                    msg.markers.push_back(mrk);
                }
            }
            shapes_pub_.publish(msg);
        }
    }
}

KDL::Frame KinematicTree::FK(KinematicFrame& frame)
{
    frame.TempA = frame.FrameA.lock()->Frame * frame.FrameAOffset;
    frame.TempB = frame.FrameB.lock()->Frame * frame.FrameBOffset;
    frame.TempAB = frame.TempB.Inverse() * frame.TempA;
    return frame.TempAB;
}

KDL::Frame KinematicTree::FK(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB)
{
    KinematicFrame frame;
    frame.FrameA = elementA;
    frame.FrameB = (elementB == nullptr) ? Root : elementB;
    frame.FrameAOffset = offsetA;
    frame.FrameBOffset = offsetB;
    return FK(frame);
}

KDL::Frame KinematicTree::FK(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB)
{
    std::string nameA = elementA == "" ? Root->Segment.getName() : elementA;
    std::string nameB = elementB == "" ? Root->Segment.getName() : elementB;
    auto A = TreeMap.find(nameA);
    if (A == TreeMap.end()) throw_pretty("Can't find link '" << nameA << "'!");
    auto B = TreeMap.find(nameB);
    if (B == TreeMap.end()) throw_pretty("Can't find link '" << nameB << "'!");
    return FK(A->second.lock(), offsetA, B->second.lock(), offsetB);
}

void KinematicTree::UpdateFK()
{
    int i = 0;
    for (KinematicFrame& frame : Solution->Frame)
    {
        Solution->Phi(i) = FK(frame);
        i++;
    }
}

Eigen::MatrixXd KinematicTree::Jacobian(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB)
{
    KinematicFrame frame;
    frame.FrameA = elementA;
    frame.FrameB = (elementB == nullptr) ? Root : elementB;
    frame.FrameAOffset = offsetA;
    frame.FrameBOffset = offsetB;
    KDL::Jacobian ret(NumControlledJoints);
    ComputeJ(frame, ret);
    return ret.data;
}

Eigen::MatrixXd KinematicTree::Jacobian(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB)
{
    std::string nameA = elementA == "" ? Root->Segment.getName() : elementA;
    std::string nameB = elementB == "" ? Root->Segment.getName() : elementB;
    auto A = TreeMap.find(nameA);
    if (A == TreeMap.end()) throw_pretty("Can't find link '" << nameA << "'!");
    auto B = TreeMap.find(nameB);
    if (B == TreeMap.end()) throw_pretty("Can't find link '" << nameB << "'!");
    return Jacobian(A->second.lock(), offsetA, B->second.lock(), offsetB);
}

void KinematicTree::ComputeJdot(KDL::Jacobian& J, KDL::Jacobian& JDot)
{
    JDot.data.setZero(J.rows(), J.columns());
    for (int i = 0; i < J.columns(); i++)
    {
        KDL::Twist tmp;
        for (int j = 0; j < J.columns(); j++)
        {
            KDL::Twist jac_i_ = J.getColumn(i);
            KDL::Twist jac_j_ = J.getColumn(j);
            KDL::Twist t_djdq_;

            if (j < i)
            {
                t_djdq_.vel = jac_j_.rot * jac_i_.vel;
                t_djdq_.rot = jac_j_.rot * jac_i_.rot;
            }
            else if (j > i)
            {
                KDL::SetToZero(t_djdq_.rot);
                t_djdq_.vel = -jac_j_.vel * jac_i_.rot;
            }
            else if (j == i)
            {
                // ref (40)
                KDL::SetToZero(t_djdq_.rot);
                t_djdq_.vel = jac_i_.rot * jac_i_.vel;
            }

            tmp += t_djdq_;
        }
        JDot.setColumn(i, tmp);
    }
}

void KinematicTree::ComputeJ(KinematicFrame& frame, KDL::Jacobian& J)
{
    J.data.setZero();
    KDL::Frame tmp = FK(frame);  // Create temporary offset frames
    std::shared_ptr<KinematicElement> it = frame.FrameA.lock();
    while (it != nullptr)
    {
        if (it->IsControlled)
        {
            KDL::Frame SegmentReference;
            if (it->Parent.lock() != nullptr) SegmentReference = it->Parent.lock()->Frame;
            J.setColumn(it->ControlId, frame.TempB.M.Inverse() * (SegmentReference.M * it->Segment.twist(TreeState(it->Id), 1.0)).RefPoint(frame.TempA.p - it->Frame.p));
        }
        it = it->Parent.lock();
    }
    it = frame.FrameB.lock();
    while (it != nullptr)
    {
        if (it->IsControlled)
        {
            KDL::Frame SegmentReference;
            if (it->Parent.lock() != nullptr) SegmentReference = it->Parent.lock()->Frame;
            J.setColumn(it->ControlId, J.getColumn(it->ControlId) - (frame.TempB.M.Inverse() * (SegmentReference.M * it->Segment.twist(TreeState(it->Id), 1.0)).RefPoint(frame.TempA.p - it->Frame.p)));
        }
        it = it->Parent.lock();
    }
}

void KinematicTree::UpdateJ()
{
    int i = 0;
    for (KinematicFrame& frame : Solution->Frame)
    {
        ComputeJ(frame, Solution->J(i));
        i++;
    }
}

void KinematicTree::UpdateJdot()
{
    int i = 0;
    for (KinematicFrame& frame : Solution->Frame)
    {
        ComputeJdot(Solution->J(i), Solution->JDot(i));
        i++;
    }
}

Eigen::MatrixXd KinematicTree::getJointLimits()
{
    Eigen::MatrixXd lim(getNumControlledJoints(), 2);
    for (int i = 0; i < ControlledJoints.size(); i++)
    {
        lim(i, 0) = ControlledJoints[i].lock()->JointLimits[0];
        lim(i, 1) = ControlledJoints[i].lock()->JointLimits[1];
    }
    return lim;
}

exotica::BASE_TYPE KinematicTree::getModelBaseType()
{
    return ModelBaseType;
}

exotica::BASE_TYPE KinematicTree::getControlledBaseType()
{
    return ControlledBaseType;
}

std::map<std::string, std::vector<double>> KinematicTree::getUsedJointLimits()
{
    std::map<std::string, std::vector<double>> limits;
    for (auto it : ControlledJoints)
    {
        limits[it.lock()->Segment.getJoint().getName()] = it.lock()->JointLimits;
    }
    return limits;
}

void KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double>& lower, const std::vector<double>& upper)
{
    if (ControlledBaseType != BASE_TYPE::FLOATING)
    {
        throw_pretty("This is not a floating joint!");
    }
    if (lower.size() != 6 || upper.size() != 6)
    {
        throw_pretty("Wrong limit data size!");
    }
    for (int i = 0; i < 6; i++)
    {
        ControlledJoints[i].lock()->JointLimits = {lower[i], upper[i]};
    }
}
void KinematicTree::setJointLimits()
{
    srdf::Model::VirtualJoint virtual_joint = Model->getSRDF()->getVirtualJoints()[0];
    std::vector<std::string> vars = Model->getVariableNames();
    for (int i = 0; i < vars.size(); i++)
    {
        if (ControlledJointsMap.find(vars[i]) != ControlledJointsMap.end())
        {
            auto& ControlledJoint = ControlledJointsMap.at(vars[i]);
            int index = ControlledJoint.lock()->ControlId;
            ControlledJoints[index].lock()->JointLimits = {Model->getVariableBounds(vars[i]).min_position_, Model->getVariableBounds(vars[i]).max_position_};
        }
    }

    ///	Manually defined floating base limits
    ///	Should be done more systematically with robot model class
    if (ControlledBaseType == BASE_TYPE::FLOATING)
    {
        ControlledJoints[0].lock()->JointLimits = {-0.05, 0.05};

        ControlledJoints[1].lock()->JointLimits = {-0.05, 0.05};

        ControlledJoints[2].lock()->JointLimits = {0.875, 1.075};

        ControlledJoints[3].lock()->JointLimits = {-0.087 / 2, 0.087 / 2};

        ControlledJoints[4].lock()->JointLimits = {-0.087 / 2, 0.2617 / 2};

        ControlledJoints[5].lock()->JointLimits = {-M_PI / 8, M_PI / 8};
    }
    else if (ControlledBaseType == BASE_TYPE::PLANAR)
    {
        ControlledJoints[0].lock()->JointLimits = {-10, 10};

        ControlledJoints[1].lock()->JointLimits = {-10, 10};

        ControlledJoints[2].lock()->JointLimits = {-1.57, 1.57};
    }
}

std::string KinematicTree::getRootFrameName()
{
    return Root->Segment.getName();
}

std::string KinematicTree::getRootJointName()
{
    return Model->getRootJoint()->getName();
}

int KinematicTree::getEffSize()
{
    return Solution->Frame.size();
}

Eigen::VectorXd KinematicTree::getModelState()
{
    Eigen::VectorXd ret(ModelJointsNames.size());

    for (int i = 0; i < ModelJointsNames.size(); i++)
    {
        ret(i) = TreeState(ModelJointsMap.at(ModelJointsNames[i]).lock()->Id);
    }
    return ret;
}

std::map<std::string, double> KinematicTree::getModelStateMap()
{
    std::map<std::string, double> ret;
    for (std::string& jointName : ModelJointsNames)
    {
        ret[jointName] = TreeState(ModelJointsMap.at(jointName).lock()->Id);
    }
    return ret;
}

void KinematicTree::setModelState(Eigen::VectorXdRefConst x)
{
    if (x.rows() != ModelJointsNames.size()) throw_pretty("Model state vector has wrong size, expected " << ModelJointsNames.size() << " got " << x.rows());
    for (int i = 0; i < ModelJointsNames.size(); i++)
    {
        TreeState(ModelJointsMap.at(ModelJointsNames[i]).lock()->Id) = x(i);
    }
    UpdateTree();
    UpdateFK();
    if (Flags & KIN_J) UpdateJ();
    if (Debug) publishFrames();
}

void KinematicTree::setModelState(std::map<std::string, double> x)
{
    for (auto& joint : x)
    {
        try
        {
            TreeState(ModelJointsMap.at(joint.first).lock()->Id) = joint.second;
        }
        catch (const std::out_of_range& e)
        {
            throw_pretty("Robot model does not contain joint '" << joint.first << "'");
        }
    }
    UpdateTree();
    UpdateFK();
    if (Flags & KIN_J) UpdateJ();
    if (Debug) publishFrames();
}

Eigen::VectorXd KinematicTree::getControlledState()
{
    Eigen::VectorXd x(NumControlledJoints);
    for (int i = 0; i < ControlledJoints.size(); i++)
    {
        x(i) = TreeState(ControlledJoints[i].lock()->Id);
    }
    return x;
}

Eigen::VectorXd KinematicTree::getControlledLinkMass()
{
    Eigen::VectorXd x(NumControlledJoints);
    for (int i = 0; i < ControlledJoints.size(); i++)
    {
        x(i) = ControlledJoints[i].lock()->Segment.getInertia().getMass();
    }
    return x;
}

std::map<std::string, shapes::ShapeType> KinematicTree::getCollisionObjectTypes()
{
    std::map<std::string, shapes::ShapeType> ret;
    for (const auto& element : CollisionTreeMap)
    {
        ret[element.second.lock()->Segment.getName()] = element.second.lock()->Shape->type;
    }
    return ret;
}

bool KinematicTree::doesLinkWithNameExist(std::string name)
{
    // Check whether it exists in TreeMap, which should encompass both EnvironmentTree and ModelTree
    if (TreeMap.find(name) != TreeMap.end())
        return true;
    else
        return false;
}
}
