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

#ifdef KIN_DEBUG_MODE
#include <iostream>
#endif

namespace exotica
{

KinematicElement::KinematicElement(int id, std::shared_ptr<KinematicElement> parent, KDL::Segment segment) : Parent(parent), Segment(segment), Id(id), IsControlled(false), ControlId(-1)
{

}

KinematicResponse::KinematicResponse() : Flags(KIN_FK)
{

}

KinematicResponse::KinematicResponse(KinematicRequestFlags flags, int size, int n)
{
    Flags = flags;
    Frame.resize(size);
    Phi.resize(size);
    if(Flags & KIN_FK_VEL) PhiDot.resize(size);
    KDL::Jacobian Jzero(n);
    Jzero.data.setZero();
    if(Flags & KIN_J) J = ArrayJacobian::Constant(size, Jzero);
    if(Flags & KIN_J_DOT) JDot = ArrayJacobian::Constant(size, Jzero);
}

KinematicsRequest::KinematicsRequest() : Flags(KIN_FK)
{
}

KinematicFrameRequest::KinematicFrameRequest()
{

}

KinematicFrameRequest::KinematicFrameRequest(std::string frameALinkName, KDL::Frame frameAOffset, std::string frameBLinkName, KDL::Frame frameBOffset) :
    FrameALinkName(frameALinkName), FrameAOffset(frameAOffset), FrameBLinkName(frameBLinkName), FrameBOffset(frameBOffset)
{

}

KinematicSolution::KinematicSolution() : Start(-1), Length(-1), Phi(nullptr,0), PhiDot(nullptr,0), J(nullptr,0), JDot(nullptr,0)
{

}

KinematicSolution::KinematicSolution(int start, int length) : Start(start), Length(length), Phi(nullptr,0), PhiDot(nullptr,0), J(nullptr,0), JDot(nullptr,0)
{
}

void KinematicSolution::Create(std::shared_ptr<KinematicResponse> solution)
{
    if(Start < 0 || Length < 0) throw_pretty("Kinematic solution was not initialized!");
    new (&Phi) Eigen::Map<ArrayFrame>(solution->Phi.data()+Start, Length);
    if(solution->Flags & KIN_FK_VEL) new (&PhiDot) Eigen::Map<ArrayTwist>(solution->PhiDot.data()+Start, Length);
    if(solution->Flags & KIN_J) new (&J) Eigen::Map<ArrayJacobian>(solution->J.data()+Start, Length);
    if(solution->Flags & KIN_J_DOT) new (&JDot) Eigen::Map<ArrayJacobian>(solution->JDot.data()+Start, Length);
}

int KinematicTree::getNumJoints()
{
  return NumControlledJoints;
}

KinematicTree::KinematicTree() : StateSize(-1), Debug(false)
{

}

void KinematicTree::Instantiate(std::string JointGroup, robot_model::RobotModelPtr model)
{
    if (!model) throw_pretty("No robot model provided!");    
    robot_model::JointModelGroup* group = model->getJointModelGroup(JointGroup);
    if(!group) throw_pretty("Joint group '"<<JointGroup<<"' not defined in the robot model!");
    ControlledJointsNames = group->getVariableNames();
    ModleJointsNames = model->getVariableNames();

    controlled_base_ = true;

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
}

void KinematicTree::BuildTree(const KDL::Tree & RobotKinematics)
{
    Tree.clear();
    TreeMap.clear();

    // Handle the root joint
    const robot_model::JointModel* RootJoint = Model->getRootJoint();
    std::string WorldFrameName;
    for(const srdf::Model::VirtualJoint& s : Model->getSRDF()->getVirtualJoints())
    {
        if(s.name_ == RootJoint->getName())
        {
            WorldFrameName = s.parent_frame_;
        }
    }
    if(WorldFrameName == "") throw_pretty("Can't initialize root joint!");
    if(RootJoint->getType()==robot_model::JointModel::FIXED)
    {
        base_type_ = BASE_TYPE::FIXED;
        Tree.push_back(std::shared_ptr<KinematicElement>(new KinematicElement(Tree.size(), nullptr, KDL::Segment(WorldFrameName, KDL::Joint(RootJoint->getName(), KDL::Joint::None))  )));
    }
    else if(RootJoint->getType() == robot_model::JointModel::FLOATING)
    {
        base_type_ = BASE_TYPE::FLOATING;
        Tree.resize(6);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::TransZ, KDL::Joint::RotX, KDL::Joint::RotY, KDL::Joint::RotZ};
        for(int i=0;i<6;i++)
        {
            Tree[i] = std::shared_ptr<KinematicElement>(new KinematicElement(i, i==0?nullptr:Tree[i-1], KDL::Segment(i==0?WorldFrameName:RootJoint->getVariableNames()[i], KDL::Joint(RootJoint->getVariableNames()[i], types[i]))  ));
            if(i>0) Tree[i-1]->Children.push_back(Tree[i]);
        }
        auto RotW = std::find(ControlledJointsNames.begin(), ControlledJointsNames.end(),RootJoint->getVariableNames()[6]);
        if(RotW!=ControlledJointsNames.end()) ControlledJointsNames.erase(RotW);
        RotW = std::find(ModleJointsNames.begin(), ModleJointsNames.end(),RootJoint->getVariableNames()[6]);
        if(RotW!=ModleJointsNames.end()) ModleJointsNames.erase(RotW);
    }
    else if(RootJoint->getType() ==  robot_model::JointModel::PLANAR)
    {
        base_type_ = BASE_TYPE::PLANAR;
        Tree.resize(3);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::RotZ};
        for(int i=0;i<3;i++)
        {
            Tree[i] = std::shared_ptr<KinematicElement>(new KinematicElement(i, i==0?nullptr:Tree[i-1], KDL::Segment(i==0?WorldFrameName:RootJoint->getVariableNames()[i], KDL::Joint(RootJoint->getVariableNames()[i], types[i]))  ));
            if(i>0) Tree[i-1]->Children.push_back(Tree[i]);
        }
    }
    else
    {
        throw_pretty("Unsupported root joint type: "<< RootJoint->getTypeName());
    }

    AddElement(RobotKinematics.getRootSegment(), *(Tree.end()-1));

    NumJoints = ModleJointsNames.size();
    NumControlledJoints = ControlledJointsNames.size();
    if (NumControlledJoints < 1) throw_pretty("No update joints specified!");
    ControlledJoints.resize(NumControlledJoints);
    Root = Tree[0];
    TreeState = Eigen::VectorXd::Zero(Tree.size());
    for(std::shared_ptr<KinematicElement> Joint : Tree)
    {
        Joint->ControlId = IsControlled(Joint);
        Joint->IsControlled = Joint->ControlId >= 0;
        if(Joint->IsControlled) ControlledJoints[Joint->ControlId] = Joint;
        TreeMap[Joint->Segment.getName()] = Joint;
        if(Joint->IsControlled) ControlledJointsMap[Joint->Segment.getJoint().getName()] = Joint;
    }
    setJointLimits();
}

void KinematicTree::AddElement(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent)
{
    std::shared_ptr<KinematicElement> NewElement(new KinematicElement(Tree.size(), parent, segment->second.segment));
    Tree.push_back(NewElement);
    if(parent) parent->Children.push_back(NewElement);
    for(KDL::SegmentMap::const_iterator child : segment->second.children)
    {
        AddElement(child, NewElement);
    }
}

int KinematicTree::IsControlled(std::shared_ptr<KinematicElement> Joint)
{
    for(int i = 0;i<ControlledJointsNames.size();i++)
    {
        if(ControlledJointsNames[i]==Joint->Segment.getJoint().getName()) return i;
    }
    return -1;
}

std::shared_ptr<KinematicResponse> KinematicTree::RequestFrames(const KinematicsRequest& request)
{
    Flags = request.Flags;
    if(Flags&KIN_J_DOT) Flags = Flags | KIN_J;
    Solution.reset(new KinematicResponse(Flags, request.Frames.size(), NumControlledJoints));

    StateSize=NumControlledJoints;
    if(((Flags&KIN_FK_VEL) || (Flags&KIN_J_DOT))) StateSize = NumControlledJoints*2;

    for(int i=0;i<request.Frames.size();i++)
    {
        if(request.Frames[i].FrameALinkName=="")
            Solution->Frame[i].FrameA = Root;
        else
            Solution->Frame[i].FrameA = TreeMap.at(request.Frames[i].FrameALinkName);

        if(request.Frames[i].FrameBLinkName=="")
            Solution->Frame[i].FrameB = Root;
        else
            Solution->Frame[i].FrameB = TreeMap.at(request.Frames[i].FrameBLinkName);

        Solution->Frame[i].FrameAOffset = request.Frames[i].FrameAOffset;
        Solution->Frame[i].FrameBOffset = request.Frames[i].FrameBOffset;
    }

    if(Debug)
    {
        for(KinematicFrame& frame : Solution->Frame)
        {
            HIGHLIGHT(frame.FrameB->Segment.getName() << " " << (frame.FrameBOffset==KDL::Frame::Identity()?"":toString(frame.FrameBOffset)) << " -> "<< frame.FrameA->Segment.getName() << " " << (frame.FrameAOffset==KDL::Frame::Identity()?"":toString(frame.FrameAOffset)));
        }
    }

    debugTree.resize(Tree.size());
    debugFrames.resize(Solution->Frame.size()*2);

    return Solution;
}

void KinematicTree::Update(Eigen::VectorXdRefConst x)
{
    if(x.rows()!=StateSize) throw_pretty("Wrong state vector size! Got " << x.rows() << " expected " << StateSize);
    UpdateTree(x);
    UpdateFK();
    if(Flags & KIN_J) UpdateJ();
    if(Debug) publishFrames();
}

void KinematicTree::UpdateTree(Eigen::VectorXdRefConst x)
{
    for(int i=0; i<ControlledJoints.size();i++)
    {
        TreeState(ControlledJoints[i]->Id) = x(i);
    }
    for(std::shared_ptr<KinematicElement> element : Tree)
    {
        KDL::Frame ParentFrame;
        if(element->Id>0) ParentFrame = element->Parent->Frame;
        element->Frame = ParentFrame * element->Segment.pose(TreeState(element->Id));
    }
}

void KinematicTree::publishFrames()
{
    int i = 0;
    for(std::shared_ptr<KinematicElement> element : Tree)
    {
        tf::Transform T;
        tf::transformKDLToTF(element->Frame, T);
        debugTree[i] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica",getRootName()), tf::resolve("exotica", element->Segment.getName()));
        i++;
    }
    debugTF.sendTransform(debugTree);
    i = 0;
    for(KinematicFrame&  frame : Solution->Frame)
    {
        tf::Transform T;
        tf::transformKDLToTF(frame.TempA, T);
        debugFrames[i*2] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica",getRootName()), tf::resolve("exotica","Frame"+std::to_string(i)+"A"+frame.FrameA->Segment.getName()));
        tf::transformKDLToTF(frame.TempB, T);
        debugFrames[i*2+1] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica",getRootName()), tf::resolve("exotica","Frame"+std::to_string(i)+"B"+frame.FrameB->Segment.getName()));
        i++;
    }
    debugTF.sendTransform(debugFrames);
}

void KinematicTree::UpdateFK()
{
    int i = 0;
    for(KinematicFrame&  frame : Solution->Frame)
    {
        frame.TempA = frame.FrameA->Frame * frame.FrameAOffset;
        frame.TempB = frame.FrameB->Frame * frame.FrameBOffset;
        frame.TempAB = frame.TempB.Inverse()*frame.TempA;
        Solution->Phi(i) = frame.TempAB;
        i++;
    }
}

void KinematicTree::ComputeJ(const KinematicFrame& frame, KDL::Jacobian& J)
{
    J.data.setZero();
    std::shared_ptr<KinematicElement> it = frame.FrameA;
    while(it!=Root)
    {
        if(it->IsControlled)
        {
            KDL::Frame JointFrame = it->Parent->Frame*KDL::Frame(it->Segment.getJoint().JointOrigin());
            J.setColumn(it->ControlId, JointFrame.M*it->Segment.getJoint().twist(1.0).RefPoint((JointFrame.Inverse()*frame.TempA).p));
        }
        it = it->Parent;
    }
    it = frame.FrameB;
    while(it!=Root)
    {
        if(it->IsControlled)
        {
            KDL::Frame JointFrame = it->Frame*it->Segment.getFrameToTip().Inverse();
            KDL::Frame PreJointFrame = it->Parent->Frame*KDL::Frame(it->Segment.getJoint().JointOrigin());
            J.setColumn(it->ControlId, J.getColumn(it->ControlId) + JointFrame.M*it->Segment.getJoint().twist(-1.0).RefPoint((PreJointFrame.Inverse()*frame.TempA).p));
        }

        it = it->Parent;
    }
    KDL::changeBase(J, frame.TempB.M, J);
}

void KinematicTree::UpdateJ()
{
    int i = 0;
    for(const KinematicFrame&  frame : Solution->Frame)
    {
        ComputeJ(frame, Solution->J(i));
        i++;
    }
}

Eigen::MatrixXd KinematicTree::getJointLimits()
{
    Eigen::MatrixXd lim(getNumJoints(),2);
    for(int i=0; i<ControlledJoints.size();i++)
    {
        lim(i,0) = ControlledJoints[i]->JointLimits[0];
        lim(i,1) = ControlledJoints[i]->JointLimits[1];
    }
    return lim;
}


void KinematicTree::updateEndEffectors(
    const KinematicsRequest & new_end_effectors)
{
// Lock for synchronisation
  boost::mutex::scoped_lock(member_lock_);

// Clear the needed flag and the end-effector segments vectors
  for (int i = 0; i < robot_tree_.size(); ++i)
  {
    robot_tree_[i].needed = false;
  }
  eff_segments_.clear();
  eff_seg_offs_.clear();

// Now do the actual updating
//  setEndEffectors(new_end_effectors);
}

bool KinematicTree::updateEndEffectorOffsets(
    const std::vector<int> & index, const std::vector<KDL::Frame> & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised()) return false;
  if (index.size() > eff_seg_offs_.size() || index.size() != offset.size())
    return false;
  for (int i = 0; i < index.size(); i++)
    eff_seg_offs_[i] = offset[i];
  return true;
}

bool KinematicTree::getEndEffectorIndex(std::vector<int> & eff_index)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised()) return false;
  eff_index = eff_segments_;
  return true;
}

exotica::BASE_TYPE KinematicTree::getBaseType()
{
  return base_type_;
}

bool KinematicTree::addEndEffector(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
// Check if the end-effector is a segment
  std::map<std::string, int>::iterator map_it = segment_map_.find(name);
  if (map_it == segment_map_.end())
  {
    return false;
  }

  uint N = eff_segments_.size(), i;
  for (i = 0; i < N; i++)
  {
    if (map_it->second == eff_segments_[i])
    {
      return false;
    }
  }
  recurseNeedFlag(segment_map_[name]);
  eff_segments_.push_back(segment_map_[name]);
  eff_seg_offs_.push_back(offset);
  forward_map_.resize(3 * eff_segments_.size()); // Just position/velocity of end-effector
  jacobian_.resize(3 * eff_segments_.size(), NumControlledJoints);
  return true;
}

bool KinematicTree::removeEndEffector(const std::string & name)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
// Check if the end-effector is a segment
  std::map<std::string, int>::iterator map_it = segment_map_.find(name);
  if (map_it == segment_map_.end()) return false;
  uint N = eff_segments_.size(), i;
  for (i = 0; i < N; i++)
  {
    if (eff_segments_[i] == map_it->second)
    {
      eff_segments_.erase(eff_segments_.begin() + i);
      eff_seg_offs_.erase(eff_seg_offs_.begin() + i);
      for (int i = 0; i < robot_tree_.size(); ++i)
      {
        robot_tree_[i].needed = false;
      }

      for (i = 0; i < N - 1; i++)
      {
        recurseNeedFlag(eff_segments_[i]);
      }
      forward_map_.resize(3 * (N - 1));	// Just position/velocity of end-effector
      jacobian_.resize(3 * (N - 1), NumControlledJoints);
      return true;
    }

  }
  return false;
}

bool KinematicTree::modifyEndEffector(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
// Check if the end-effector is a segment
  std::map<std::string, int>::iterator map_it = segment_map_.find(name);
  if (map_it == segment_map_.end()) return false;
  uint index = map_it->second;
  uint N = eff_segments_.size(), i;
  for (i = 0; i < N; i++)
  {
    if (eff_segments_[i] == index)
    {
      eff_seg_offs_[i] = offset;
      return true;
    }
  }
  return false;
}

bool KinematicTree::modifySegment(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
// Check if the end-effector is a segment
  std::map<std::string, int>::iterator map_it = segment_map_.find(name);
  if (map_it == segment_map_.end()) return false;
  robot_tree_[map_it->second].offset = offset;
  return true;
}

bool KinematicTree::updateConfiguration(
    const Eigen::Ref<const Eigen::VectorXd> & joint_configuration)
{
    return true; // Temporary
// Temporaries
  double jnt_angle;

  // Locking
  boost::mutex::scoped_lock(member_lock_);

// Checks
  if (!isInitialised())
  {
    ERROR("Scene was not initialized!");
    return false;
  }
  if (!zero_undef_jnts_ && joint_configuration.size() != NumControlledJoints)
  {
    ERROR(
        "Joint vector size is incorrect!\nExpected "<<joint_configuration.size()<<", found "<<NumControlledJoints);
    return false;
  }

// Start update: start first with the root node...
  jnt_angle =
      (robot_tree_[0].joint_type == JNT_UNUSED) ?
          0 : joint_configuration[robot_tree_[0].joint_index];
  robot_tree_[0].joint_pose = robot_tree_[0].tip_pose
      * robot_tree_[0].segment.pose(jnt_angle).Inverse();
  if (robot_tree_[0].joint_type)	// Will be greater than 0
  {
    robot_tree_[0].joint_origin = vectorKdlToEigen(
        robot_tree_[0].joint_pose
            * robot_tree_[0].segment.getJoint().JointOrigin());
    robot_tree_[0].joint_axis = vectorKdlToEigen(
        robot_tree_[0].joint_pose.M
            * robot_tree_[0].segment.getJoint().JointAxis());
    if (!robot_tree_[0].to_tip)
    {
      robot_tree_[0].joint_axis = -1.0 * robot_tree_[0].joint_axis;
    }
  }

  for (int i = 1; i < robot_tree_.size(); i++)
  {
    if (robot_tree_[i].needed)	// Only proceed if needed
    {
      // Temporaries
      KDL::Frame parent_transform;

      // Settle Angle
      if (!controlled_base_ && i < 4)
      {
        if (i == 1) jnt_angle = current_base_pose_.p.data[0];
        if (i == 2) jnt_angle = current_base_pose_.p.data[1];
        if (i == 3)
        {
          KDL::Vector axis(0, 0, 1);
          jnt_angle = current_base_pose_.M.GetRotAngle(axis);
        }
      }
      else
        jnt_angle =
            (robot_tree_[i].joint_type == JNT_UNUSED) ?
                0 : joint_configuration[robot_tree_[i].joint_index];

      // Settle which parent transform to use
      if (robot_tree_[i].from_tip)//If we are coming from the tip of the parent
      {
        parent_transform = robot_tree_[robot_tree_[i].parent].tip_pose;
      }
      else
      {
        parent_transform = robot_tree_[robot_tree_[i].parent].joint_pose;
      }

      // Now settle the tip or base
      if (robot_tree_[i].to_tip)	// We are moving towards the tip
      {	// We generally do not need to concern ourselves with the joint_pose: however might need it for the joint origin computation
        robot_tree_[i].tip_pose = parent_transform
            * robot_tree_[i].segment.pose(jnt_angle) * robot_tree_[i].offset;
        robot_tree_[i].joint_pose = parent_transform * robot_tree_[i].offset;
      }
      else // Moving towards the base
      {
        robot_tree_[i].tip_pose = parent_transform * robot_tree_[i].offset;	// We cannot be moving towards base from a tip
        robot_tree_[i].joint_pose = parent_transform
            * robot_tree_[i].segment.pose(jnt_angle).Inverse()
            * robot_tree_[i].offset;
      }

      // Finally set the joint_origin/axis
      if (robot_tree_[i].joint_type)// If we are specifying this joint: if not, it does not make sense to compute it
      {
        if (robot_tree_[i].joint_type == JNT_ROTARY) // Only compute axis if rotary
        {
          robot_tree_[i].joint_origin = vectorKdlToEigen(
              robot_tree_[i].joint_pose
                  * robot_tree_[i].segment.getJoint().JointOrigin()); // Origin is just transformed into the global frame
        }
        robot_tree_[i].joint_axis = vectorKdlToEigen(
            robot_tree_[i].joint_pose.M
                * robot_tree_[i].segment.getJoint().JointAxis()); // For the axes, we do not care about co-ordinates: we just pre-multiply by rotation: this is needed always, for both rotary and prismatic joints
        if (!robot_tree_[i].to_tip)
        {
          robot_tree_[i].joint_axis = -1.0 * robot_tree_[i].joint_axis;
        }
      }
    }
  }
  return true;
}

bool KinematicTree::setBasePose(const KDL::Frame &pose)
{
  current_base_pose_ = pose;
  return true;
}

void KinematicTree::setBaseBounds(const std::vector<double> &bounds)
{
  if (bounds.size() != 6)
  {
    throw_pretty("Expect base bounds size 6, but received bounds size "<<bounds.size());
  }

  if (base_type_ == BASE_TYPE::FLOATING)
  {
    //Here we are just setting the bounds of allowed base movement
    ControlledJoints[0]->JointLimits = {bounds[0], bounds[3]};
    ControlledJoints[1]->JointLimits = {bounds[1], bounds[4]};
    ControlledJoints[2]->JointLimits = {bounds[2], bounds[5]};
  }
}

bool KinematicTree::generateForwardMap()
{
  boost::mutex::scoped_lock(member_lock_);	// Lock:
  return computePhi();
}

bool KinematicTree::generateForwardMap(Eigen::Ref<Eigen::VectorXd> phi)
{
    return true; // Temporary
  boost::mutex::scoped_lock(member_lock_);	// Lock for thread-safety

  if (computePhi())	// If successful:
  {
    if (phi.rows() != forward_map_.rows())
    {
      ERROR(
          "Return vector has wrong size! Has "<<forward_map_.rows()<<", required "<<phi.rows());
      return false;
    }
    else
    {
      phi = forward_map_;
      return true;
    }
  }
  else
  {
    INDICATE_FAILURE
    ;
    return false;
  }
}

bool KinematicTree::getPhi(Eigen::Ref<Eigen::VectorXd> phi)
{
  boost::mutex::scoped_lock(member_lock_);	// Lock for thread-safety
  if (phi.rows() != forward_map_.rows())
  {
    ERROR(
        "Return vector has wrong size ("<<phi.rows()<<"!="<<forward_map_.rows()<<")!");
    return false;
  }
  phi = forward_map_;
  return true;
}

bool KinematicTree::generateJacobian()
{
  boost::mutex::scoped_lock(member_lock_); 	// Locking
  return computePosJacobian();
}

bool KinematicTree::getJacobian(Eigen::Ref<Eigen::MatrixXd> jac)
{
  boost::mutex::scoped_lock(member_lock_);	// Lock for thread-safety
  if (jac.rows() != jacobian_.rows() || jac.cols() != jacobian_.cols())
  {
    std::cout << "Has " << jacobian_.rows() << "X" << jacobian_.cols()
        << ". Req: " << jac.rows() << "x" << jac.cols() << std::endl;
    ERROR("Return matrix has wrong size!");
    return false;
  }
  jac = jacobian_;
  return true;
}

bool KinematicTree::generateJacobian(
    Eigen::Ref<Eigen::MatrixXd> jacobian)
{
    return true; // Temporary
  boost::mutex::scoped_lock(member_lock_); 	// Locking

  if (computePosJacobian()) 	// If ok...
  {
    if (jacobian.rows() != jacobian_.rows()
        || jacobian.cols() != jacobian_.cols())
    {
      ERROR(
          "Return matrix has wrong size! Required size "<<jacobian.rows()<<"x"<<jacobian.cols()<<". Has size"<<jacobian_.rows()<<"x"<<jacobian_.cols());
      return false;
    }
    jacobian = jacobian_;
    return true;
  }
  else
  {
    INDICATE_FAILURE
    ;
    return false;
  }
}

bool KinematicTree::generateCoM()
{

  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  uint N = robot_tree_.size(), i;
  double M = 0, m;
  KDL::Vector com = KDL::Vector::Zero();
  KDL::Frame tip_pose;
  for (i = 0; i < N; i++)
  {
    m = robot_tree_[i].segment.getInertia().getMass();
    M = M + m;
    tip_pose = robot_tree_[i].joint_pose
        * KDL::Frame(robot_tree_[i].segment.getInertia().getCOG());
    com = com + m * tip_pose.p;
  }

  com = com / M;
  com_.x() = com.x();
  com_.y() = com.y();
  com_.z() = com.z();
  return true;
}
bool KinematicTree::getCoMProperties(const std::vector<int>& ids,
    std::vector<std::string> & segs, Eigen::VectorXd & mass,
    std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
    std::vector<KDL::Frame> & base_pose)
{
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  int i;
  mass.resize(ids.size());
  cog.resize(ids.size());
  tip_pose.resize(ids.size());
  base_pose.resize(ids.size());
  segs.resize(ids.size());
  for (i = 0; i < ids.size(); i++)
  {
    segs[i] = robot_tree_[ids[i]].segment.getName();
    mass(i) = robot_tree_[ids[i]].segment.getInertia().getMass();
    cog[i] = robot_tree_[ids[i]].segment.getInertia().getCOG();
    tip_pose[i] = robot_tree_[ids[i]].tip_pose;
    base_pose[i] = robot_tree_[ids[i]].joint_pose;
  }
  return true;
}

bool KinematicTree::getSegment(KDL::Segment & seg, int index)
{
  boost::mutex::scoped_lock(member_lock_);
  if (index < 0 || index >= robot_tree_.size() || !isInitialised())
  {
    return false;	// Not initialised, or invalid index
  }
  seg = robot_tree_[index].segment;
  return true;
}

bool KinematicTree::getSegments(std::vector<KDL::Segment> & segs)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
  uint N = robot_tree_.size(), i;
  segs.resize(N);
  for (i = 0; i < N; i++)
  {
    segs[i] = robot_tree_[i].segment;
  }
  return true;
}

bool KinematicTree::getControlledSegmentsAndJoints(
    std::vector<std::string> & segs, std::vector<std::string> & joints)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
  segs.resize(used_joints_segs_.size());
  segs = used_joints_segs_;
  joints.resize(NumControlledJoints);
  joints = used_joints_;
  return true;
}

bool KinematicTree::getInitialEff(std::vector<std::string> & segs,
    std::vector<KDL::Frame> & offsets)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
  segs.resize(eff_segments_ini_.size());
  offsets.resize(eff_segments_ini_.size());
  segs = eff_segments_ini_;
  offsets = eff_seg_offs_ini_;
  return true;
}

void KinematicTree::buildTree(const KDL::Tree & temp_tree,
    std::map<std::string, int> & joint_map)
{
  INFO("buildTree Function ... ");

// Variable Declarations
  KDL::SegmentMap::const_iterator root_segment; // Root segment iterator
  std::string true_root; // The urdf root name
  int rubbish; // Garbage value since we know root will be at 0

  INFO("buildTree Function ... root is of size 0");
  root_segment = temp_tree.getRootSegment();

  robot_root_.first = root_segment->first;

// if using floating base
  if (base_type_ != BASE_TYPE::FIXED)
  {
    if (Model->getSRDF()->getVirtualJoints().size() != 1)
    {
      throw_pretty(Model->getSRDF()->getVirtualJoints().size()<<" virtual joints are defined, must be set to 1. Can not use floating base");
    }
    else
    {
      //	Tricky part, all we need is to create another tree with the floating base and append the robot tree to it
      srdf::Model::VirtualJoint virtual_joint =
          Model->getSRDF()->getVirtualJoints()[0];
      if (virtual_joint.child_link_ != root_segment->first)
      {
        throw_pretty("Virtual joint has child link "<<virtual_joint.child_link_<<", but robot root is "<<root_segment->first);
      }
      std::string world = virtual_joint.parent_frame_;
      std::string world_joint = virtual_joint.name_;
      KDL::Tree base_tree(world);

      if (base_type_ == BASE_TYPE::FLOATING)
      {
        //	Naming based on moveit::core::FloatingJointModel
        //	http://docs.ros.org/indigo/api/moveit_core/html/floating__joint__model_8cpp_source.html
        //	Add translation X
        KDL::Segment transX_seg(world + "/trans_x",
            KDL::Joint(world_joint + "/trans_x", KDL::Joint::TransX));
        if (!base_tree.addSegment(transX_seg, world))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add translation Y
        KDL::Segment transY_seg(world + "/trans_y",
            KDL::Joint(world_joint + "/trans_y", KDL::Joint::TransY));
        if (!base_tree.addSegment(transY_seg, world + "/trans_x"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add translation Z
        KDL::Segment transZ_seg(world + "/trans_z",
            KDL::Joint(world_joint + "/trans_z", KDL::Joint::TransZ));
        if (!base_tree.addSegment(transZ_seg, world + "/trans_y"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add rotation X
        KDL::Segment rotX_seg(world + "/rot_x",
            KDL::Joint(world_joint + "/rot_x", KDL::Joint::RotX));
        if (!base_tree.addSegment(rotX_seg, world + "/trans_z"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add rotation Y
        KDL::Segment rotY_seg(world + "/rot_y",
            KDL::Joint(world_joint + "/rot_y", KDL::Joint::RotY));
        if (!base_tree.addSegment(rotY_seg, world + "/rot_x"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add rotation Z (which should be named as robot tree's root)
        KDL::Segment rotZ_seg(root_segment->first,
            KDL::Joint(root_segment->first + "/virtual_joint",
                KDL::Joint::RotZ));
        if (!base_tree.addSegment(rotZ_seg, world + "/rot_y"))
        {
          throw_pretty("Can't add virtual joint!");
        }

        robot_root_.second = 6;
      }
      else if (base_type_ == BASE_TYPE::PLANAR)
      {
        KDL::Segment X_seg(world + "/x",
            KDL::Joint(world_joint + "/x", KDL::Joint::TransX));
        if (!base_tree.addSegment(X_seg, world))
        {
          throw_pretty("Can't add virtual joint!");
        }
        //	Add translation Y
        KDL::Segment Y_seg(world + "/y",
            KDL::Joint(world_joint + "/y", KDL::Joint::TransY));
        if (!base_tree.addSegment(Y_seg, world + "/x"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        KDL::Segment rot_seg(root_segment->first,
            KDL::Joint(root_segment->first + "/virtual_joint",
                KDL::Joint::RotZ));
        if (!base_tree.addSegment(rot_seg, world + "/y"))
        {
          throw_pretty("Can't add virtual joint!");
        }
        robot_root_.second = 3;
      }
      if (base_type_==BASE_TYPE::FIXED)
      {
        addSegment(temp_tree.getRootSegment(), ROOT, rubbish, true, false, world, joint_map);
      }
      else if (base_tree.addTree(temp_tree, root_segment->first))
      {
        addSegment(base_tree.getRootSegment(), ROOT, rubbish, true, false, world, joint_map);
      }
      else
      {
        throw_pretty("Cant initialise KDL tree!");
      }

    }

  }
  else
  {
    robot_root_.second = 0;
    true_root = temp_tree.getRootSegment()->second.segment.getName();
    addSegment(root_segment, ROOT, rubbish, true, false, true_root,joint_map); // We do a little trick here to indicate that this is the root node
  }
}

std::map<std::string, int> KinematicTree::getJointMap()
{
  return joint_map_;
}

std::map<std::string, std::vector<double>> KinematicTree::getUsedJointLimits()
{
  std::map<std::string, std::vector<double>> limits;
  for (std::shared_ptr<KinematicElement> it : ControlledJoints)
  {
    limits[it->Segment.getJoint().getName()] = it->JointLimits;
  }
  return limits;
}

KDL::Frame KinematicTree::getRobotRootWorldTransform()
{
  KDL::Frame trans = KDL::Frame::Identity();
  if (base_type_ == BASE_TYPE::FIXED)
    return trans;
  else
    getPose(robot_root_.second, trans);
  return trans;
}

void KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double> & lower, const std::vector<double> & upper)
{
  if (base_type_ != BASE_TYPE::FLOATING)
  {
    throw_pretty("This is not a floating joint!");
  }
  if (lower.size() != 6 || upper.size() != 6)
  {
    throw_pretty("Wrong limit data size!");
  }
  for (int i = 0; i < 6; i++)
  {
    ControlledJoints[i]->JointLimits = {lower[i], upper[i]};
  }
}
void KinematicTree::setJointLimits()
{
  srdf::Model::VirtualJoint virtual_joint =  Model->getSRDF()->getVirtualJoints()[0];
  std::vector<std::string> vars = Model->getVariableNames();
  for (int i = 0; i < vars.size(); i++)
  {
    if (ControlledJointsMap.find(vars[i]) != ControlledJointsMap.end())
    {
      int index = ControlledJointsMap.at(vars[i])->ControlId;
      ControlledJoints[index]->JointLimits = {Model->getVariableBounds(vars[i]).min_position_, Model->getVariableBounds(vars[i]).max_position_};
    }
  }

///	Manually defined floating base limits
///	Should be done more systematically with robot model class
  if (base_type_ == BASE_TYPE::FLOATING)
  {
    ControlledJoints[0]->JointLimits = {-0.05, 0.05};

    ControlledJoints[1]->JointLimits = {-0.05, 0.05};

    ControlledJoints[2]->JointLimits = {0.875, 1.075};

    ControlledJoints[3]->JointLimits = {-0.087 / 2, 0.087 / 2};

    ControlledJoints[4]->JointLimits = {-0.087 / 2, 0.2617 / 2};

    ControlledJoints[5]->JointLimits = {-M_PI / 8, M_PI / 8};
  }
  else if (base_type_ == BASE_TYPE::PLANAR)
  {
    ControlledJoints[0]->JointLimits = {-10, 10};

    ControlledJoints[1]->JointLimits = {-10, 10};

    ControlledJoints[2]->JointLimits = {-1.57, 1.57};
  }
}

void KinematicTree::setJointOrder(
    const std::vector<std::string> & joints, bool zero_out,
    const std::map<std::string, int> & joint_map)
{
// First do some checks
  if (!zero_out && joints.size() != joint_map.size())
  {
    throw_pretty("Empty joint list/map!");
  }

// Now fill in the joints that will be specified: rest will still be invalidated
  NumControlledJoints = joints.size();
  used_joints_.resize(NumControlledJoints);
  used_joints_segs_.resize(NumControlledJoints);
  used_joints_ = joints;
  for (int i = 0; i < NumControlledJoints; i++)
  {
    if (joint_map.find(joints[i]) == joint_map.end()) // Guard against possibility of specifying incorrect joints
    {
      throw_pretty("buildTree Function ...  could not find joint "<<joints[i]);
    }
    // First check what type of joint it is:
    switch (robot_tree_[joint_map.at(joints[i])].segment.getJoint().getType())
    {
    case KDL::Joint::RotAxis:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_ROTARY;
      break;
    case KDL::Joint::RotX:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_ROTARY;
      break;
    case KDL::Joint::RotY:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_ROTARY;
      break;
    case KDL::Joint::RotZ:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_ROTARY;
      break;

    case KDL::Joint::TransAxis:
    case KDL::Joint::TransX:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_PRISMATIC;
      break;
    case KDL::Joint::TransY:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_PRISMATIC;
      break;
    case KDL::Joint::TransZ:
      robot_tree_[joint_map.at(joints[i])].joint_type = JNT_PRISMATIC;
      break;

    default:
      throw_pretty("Unknown joint type: "<<robot_tree_[joint_map.at(joints[i])].joint_type);
    }
    // Now set the joint index
    robot_tree_[joint_map.at(joints[i])].joint_index = i;
    used_joints_segs_[i] =
        robot_tree_[joint_map.at(joints[i])].segment.getName();
  }
}

std::string KinematicTree::getRootName()
{
  return Tree[0]->Segment.getName();
}

bool KinematicTree::modifyRootOffset(KDL::Frame & offset)
{
  if (!isInitialised())
  {
    ERROR("Kinematic tree was not initialized!");
    return false;
  }
  robot_tree_[0].tip_pose = offset.Inverse();
  return true;
}

KDL::Frame KinematicTree::getRootOffset()
{
  return robot_tree_[0].tip_pose.Inverse();
}

void KinematicTree::addSegment(
    KDL::SegmentMap::const_iterator current_segment, int parent, int & current,
    bool from_ptip, bool to_ctip, const std::string & root_name,
    std::map<std::string, int> & joint_map)
{
// Variable Declaration
  KinematicElement_t current_node;

  INFO("addSegment Function ... with " << current_segment->second.segment.getName() << " parent: " << parent << " flags: " << from_ptip << to_ctip << " root: " << root_name);

// First fill in this node
  current_node.parent = parent; // Assign the parent
  current_node.from_tip = from_ptip; // Indicate whether we reached this through the tip (true) or the base of the parent
  current_node.to_tip = to_ctip; // Also check whether we are moving towards the tip or the base
  current_node.segment = current_segment->second.segment; // Assign the segment information
  current_node.joint_type = JNT_UNUSED;
  current_node.needed = false; // By default not needed
  current = robot_tree_.size(); // Set where this node will be stored
  robot_tree_.push_back(current_node); // Store

  INFO("addSegment Function ... created node and pushed back on tree at " << robot_tree_.size() - 1);
// Update the Segment Map and the Joint Map:
  segment_map_[current_node.segment.getName()] = current;
  joint_map[current_node.segment.getJoint().getName()] = current;

  INFO("addSegment Function ... Indexing Segment and joint maps (" << current_node.segment.getJoint().getName() << ")");

// Now comes the tricky part:
  if (to_ctip) // We are moving in the forward direction towards the tip (this was a child of the node calling the function)
  {
    INFO("addSegment Function ... Moving to a tip ");
    // First Iterate through children
    for (int i = 0; i < current_segment->second.children.size(); i++) // Iterate through the children if any
    {
      INFO("addSegment Function ... Iterating through children: " << i);
      int child;
      addSegment(current_segment->second.children[i], current, child,
          true, true, root_name, joint_map); // We are moving from tip towards a tip
      robot_tree_[current].child.push_back(child); // Assign Child to this node
    }
    // Base Case: If empty, loop will be skipped
  }
  else // We are moving towards the base
  {
    INFO("addSegment Function ... Moving to a base ");
    if (from_ptip) // This combination (from tip but moving to a base) is impossible, but is used to indicate this is the root node
    {
      INFO("addSegment Function ... Manipulating Root segment ");
      // Iterate first through children
      for (int i = 0; i < current_segment->second.children.size();
          i++) // Iterate through the children if any
      {
        INFO("addSegment Function ... Iterating through children of root: " << i);
        int child;
        addSegment(current_segment->second.children[i], current,
            child, true, true, root_name, joint_map); // We are moving from tip towards a tip
        robot_tree_[current].child.push_back(child); // Assign Child to this node
      }
      // Now handle the parent: only if previously successfull and if this is not the original root node
      if (root_name!=current_node.segment.getName())
      {
        INFO("addSegment Function ... Checking parent of root ");
        int child;
        addSegment(current_segment->second.parent, current, child,
            false, false, root_name, joint_map); // We are moving from base towards base
        robot_tree_[current].child.push_back(child);	// Add as child
      }
    }
    else	// I.e. we Are moving from base to a base
    {
      INFO("addSegment Function ... Moving from base to base: ");
      // Iterate through children and set them as children of parent rather than current
      for (int i = 0; i < current_segment->second.children.size();
          i++)
      {
        INFO("addSegment Function ... Iterating through children of an inverted segment: " << i);
        int child;
        std::string child_name =
            current_segment->second.children[i]->second.segment.getName();// The name of this child
        std::string parent_name = robot_tree_[parent].segment.getName();// The name of the parent
        if (parent_name==child_name)
        {
          continue;
        }									// Skip the child who is now parent
        addSegment(current_segment->second.children[i], parent, child,
            false, true, root_name, joint_map);
        robot_tree_[parent].child.push_back(child);	// Assign child to parent node
      }
      // If empty, loop will be skipped
      if (root_name!=current_node.segment.getName())	// IF not equal to the root
      {
        INFO("addSegment Function ... Handling parent of inverted segment: ");
        int child;
        addSegment(current_segment->second.parent, current, child,
            false, false, root_name, joint_map); // Add its parent as its child, but indicate so in the traversal direction
        robot_tree_[current].child.push_back(child);
      }
      // Base case if this is indeed the root node in the original urdf
    }
  }
}

void KinematicTree::recurseNeedFlag(int node)
{
    robot_tree_[node].needed = true;
    if (robot_tree_[node].parent == ROOT)
        robot_tree_[robot_tree_[node].parent].needed = true;
    else
        recurseNeedFlag(robot_tree_[node].parent);
}

int KinematicTree::getEffSize()
{
  return Solution->Frame.size();
}

bool KinematicTree::computePhi()
{
// Checks
  if (!isInitialised())
  {
    ERROR("Kinematic tree was not initialized!");
    return false;
  }

  for (int i = 0; i < eff_segments_.size(); i++)
  {
    KDL::Frame end_effector = robot_tree_[eff_segments_[i]].tip_pose;	// End effector is w.r.t. tip of segment always
    if (eff_seg_offs_.size())	// if Size is greater than 0
    {
      end_effector = end_effector * eff_seg_offs_[i];	// Append the respective final transformation
    }
    forward_map_(i * 3) = end_effector.p.x();
    forward_map_(i * 3 + 1) = end_effector.p.y();
    forward_map_(i * 3 + 2) = end_effector.p.z();
  }
  return true;
}

bool KinematicTree::computePosJacobian()
{
// Checks
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }		// Ensure that Variables are initialised
  if (!forward_map_.size())
  {
    INDICATE_FAILURE
    ;
    return false;
  }	// Ensure that forward_map_ is generated

// Compute Jacobian for each end-effector:
  jacobian_.fill(0.0);	// Reset everything to 0
  for (int i = 0; i < eff_segments_.size(); i++)// Iterate through the end-effectors
  {
    // Temporaries
    Eigen::Vector3d end_effector_pos = forward_map_.segment(i * 3, 3);// Global End-effector position
    int segment_index = eff_segments_[i];	// Current segment index
    while (robot_tree_[segment_index].parent != ROOT)	// Repeat until you reach the root joint
    {
      // Some tricks: basically, in some cases we may need to execute for both...
      if (robot_tree_[segment_index].to_tip)// If in traversing through node we go from base to tip, then we need to consider the effect of its joint
      {
        if (robot_tree_[segment_index].joint_type == JNT_ROTARY)
        {
          Eigen::Vector3d diff_vector = end_effector_pos
              - robot_tree_[segment_index].joint_origin;	// Find the p-vector
          jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
              robot_tree_[segment_index].joint_axis.cross(diff_vector);	// The Jacobian for this joint
        }
        else if (robot_tree_[segment_index].joint_type == JNT_PRISMATIC)
        {
          jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
              robot_tree_[segment_index].joint_axis; // Just the axis
        }
        // Else will not be considered
      }
      if (!robot_tree_[segment_index].from_tip) // If we are connected to parent from its base, then we (also) need to consider the parents' joint, which also moves us...
      {
        if (robot_tree_[robot_tree_[segment_index].parent].joint_type
            == JNT_ROTARY)
        {
          Eigen::Vector3d diff_vector = end_effector_pos
              - robot_tree_[robot_tree_[segment_index].parent].joint_origin; // Find the p-vector (now with its parent joint)
          jacobian_.block(i * 3,
              robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
              robot_tree_[robot_tree_[segment_index].parent].joint_axis.cross(
                  diff_vector); // The Jacobian for this joint
        }
        else if (robot_tree_[robot_tree_[segment_index].parent].joint_type
            == JNT_PRISMATIC)
        {
          jacobian_.block(i * 3,
              robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
              robot_tree_[robot_tree_[segment_index].parent].joint_axis;
        }
      }
      segment_index = robot_tree_[segment_index].parent; // Move to its parent
    }
  }

// If made it this far:
  return true;
}

bool KinematicTree::getPose(std::string child, std::string parent,
    KDL::Frame & pose)
{
// Synchronisation
  boost::mutex::scoped_lock(member_lock_);

// Checks
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (segment_map_.find(child) == segment_map_.end())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (segment_map_.find(parent) == segment_map_.end())
  {
    INDICATE_FAILURE
    ;
    return false;
  }

// Computation
  pose = robot_tree_[segment_map_[parent]].tip_pose.Inverse()
      * robot_tree_[segment_map_[child]].tip_pose;

// Return
  return true;
}

bool KinematicTree::getPose(std::string child, KDL::Frame & pose)
{
// Synchronisation
  boost::mutex::scoped_lock(member_lock_);

// Checks
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (segment_map_.find(child) == segment_map_.end())
  {
    INDICATE_FAILURE
    ;
    return false;
  }

// Computation

  pose = robot_tree_[segment_map_[child]].tip_pose;
// Return
  return true;
}

bool KinematicTree::getPose(int child, int parent, KDL::Frame & pose)
{
  boost::mutex::scoped_lock(member_lock_);

  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (child < 0 || child > robot_tree_.size())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (parent < 0 || parent > robot_tree_.size())
  {
    INDICATE_FAILURE
    ;
    return false;
  }

  pose = robot_tree_[parent].tip_pose.Inverse() * robot_tree_[child].tip_pose;

  return true;
}

bool KinematicTree::getPose(int child, KDL::Frame & pose)
{
  boost::mutex::scoped_lock(member_lock_);

  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }
  if (child < 0 || child > robot_tree_.size())
  {
    INDICATE_FAILURE
    ;
    return false;
  }

  pose = robot_tree_[child].tip_pose;

  return true;
}

bool KinematicTree::getSegmentMap(
    std::map<std::string, int> & segment_map)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }

  segment_map = segment_map_;
  return true;

}

std::string KinematicTree::getParent(std::string child)
{
  boost::mutex::scoped_lock(member_lock_);

  if (segment_map_.find(child) != segment_map_.end())
  {
    return robot_tree_[robot_tree_[segment_map_[child]].parent].segment.getName();
  }
  else
  {
    return "";  // Empty string
  }
}

int KinematicTree::getParent(int child)
{
  boost::mutex::scoped_lock(member_lock_);

  if (child < robot_tree_.size() && child >= 0)
  {
    return robot_tree_[child].parent;
  }
  else
  {
    return -2;
  }
}

std::vector<std::string> KinematicTree::getChildren(std::string parent)
{
  boost::mutex::scoped_lock(member_lock_);

  std::vector<std::string> children;
  if (segment_map_.find(parent) != segment_map_.end())
  {
    int num_chldrn = robot_tree_[segment_map_[parent]].child.size();
    for (int i = 0; i < num_chldrn; i++)
    {
      children.push_back(
          robot_tree_[robot_tree_[segment_map_[parent]].child[i]].segment.getName());
    }
  }
  return children; // Which may be an empty array
}

std::vector<int> KinematicTree::getChildren(int parent)
{
  boost::mutex::scoped_lock(member_lock_);

  if (parent < robot_tree_.size() && parent >= 0)
  {
    return robot_tree_[parent].child;
  }
  else
  {
    std::vector<int> children;
    return children;
  }
}

Eigen::Vector3d vectorKdlToEigen(const KDL::Vector & kdl_vec)
{
  Eigen::Vector3d eigen_vec;

  eigen_vec(0) = kdl_vec.x();
  eigen_vec(1) = kdl_vec.y();
  eigen_vec(2) = kdl_vec.z();

  return eigen_vec;
}

bool recursivePrint(KinematicTree & robot, std::string node,
    std::string tab)
{
  if (tab.size() == 0)
  {
    std::cout << node << std::endl;
  }
  else if (tab.back() == '|')
  {
    std::cout << tab << "-->" << node << std::endl;
  }
  else
  {
    std::cout << tab << "|-->" << node << std::endl;
  }

  std::vector<std::string> children = robot.getChildren(node);

  if (children.size() == 0)
  {
    return true;
  }

  tab.append("  ");
  if (children.size() > 1)
  {
    tab.append("|");
  }
  else
  {
    tab.append("  ");
  }

  bool success = true;
  for (int i = 0; i < children.size() && success; i++)
  {
    if (i == children.size() - 1) // if last element...
    {
      tab.resize(tab.size() - 1);
    }
    success = recursivePrint(robot, children[i], tab);
  }
  return success;
}

}
