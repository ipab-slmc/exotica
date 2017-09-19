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

#include <kdl/frames_io.hpp>

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
    ModelJointsNames = model->getVariableNames();

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
    ModelJointsMap.clear();

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
        ModelBaseType = BASE_TYPE::FIXED;
        Tree.push_back(std::shared_ptr<KinematicElement>(new KinematicElement(Tree.size(), nullptr, KDL::Segment(WorldFrameName, KDL::Joint(RootJoint->getName(), KDL::Joint::None))  )));
    }
    else if(RootJoint->getType() == robot_model::JointModel::FLOATING)
    {
        ModelBaseType = BASE_TYPE::FLOATING;
        Tree.resize(6);
        KDL::Joint::JointType types[] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::TransZ, KDL::Joint::RotX, KDL::Joint::RotY, KDL::Joint::RotZ};
        for(int i=0;i<6;i++)
        {
            Tree[i] = std::shared_ptr<KinematicElement>(new KinematicElement(i, i==0?nullptr:Tree[i-1], KDL::Segment(i==0?WorldFrameName:RootJoint->getVariableNames()[i], KDL::Joint(RootJoint->getVariableNames()[i], types[i]))  ));
            if(i>0) Tree[i-1]->Children.push_back(Tree[i]);
        }
        auto RotW = std::find(ControlledJointsNames.begin(), ControlledJointsNames.end(),RootJoint->getVariableNames()[6]);
        if(RotW!=ControlledJointsNames.end()) ControlledJointsNames.erase(RotW);
        RotW = std::find(ModelJointsNames.begin(), ModelJointsNames.end(),RootJoint->getVariableNames()[6]);
        if(RotW!=ModelJointsNames.end()) ModelJointsNames.erase(RotW);
    }
    else if(RootJoint->getType() ==  robot_model::JointModel::PLANAR)
    {
        ModelBaseType = BASE_TYPE::PLANAR;
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
    ModelTree = Tree;

    UpdateModel();

    NumJoints = ModelJointsNames.size();
    NumControlledJoints = ControlledJointsNames.size();
    if (NumControlledJoints < 1) throw_pretty("No update joints specified!");
    ControlledJoints.resize(NumControlledJoints);
    for(std::shared_ptr<KinematicElement> Joint : Tree)
    {
        Joint->ControlId = IsControlled(Joint);
        Joint->IsControlled = Joint->ControlId >= 0;
        if(Joint->IsControlled) ControlledJoints[Joint->ControlId] = Joint;
        ModelJointsMap[Joint->Segment.getJoint().getName()] = Joint;
        if (Joint->IsControlled)
        {
          ControlledJointsMap[Joint->Segment.getJoint().getName()] = Joint;

          // The ModelBaseType defined above refers to the base type of the
          // overall robot model - not of the set of controlled joints. E.g. a
          // floating-base robot can have a scene defined where the
          // floating-base virtual joint is _not_ part of the planning
          // group/scene. Thus we need to establish the BaseType of the joint
          // group, the ControlledBaseType - if a controlled joint corresponds
          // to a floating base joint, the ControlledBaseType is the same as the
          // ModelBaseType.
          if (Joint->Segment.getJoint().getName() ==
              RootJoint->getName() + "/trans_x")
            ControlledBaseType = ModelBaseType;
        }
    }

    setJointLimits();
}

void KinematicTree::UpdateModel()
{
    Root = Tree[0];
    TreeState = Eigen::VectorXd::Zero(Tree.size());
    for(std::shared_ptr<KinematicElement> Joint : Tree)
    {
        TreeMap[Joint->Segment.getName()] = Joint;
    }
    debugTree.resize(Tree.size()-1);
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
            try 
            {
                Solution->Frame[i].FrameA = TreeMap.at(request.Frames[i].FrameALinkName);    
            }
            catch (const std::out_of_range& e)
            {
                throw_pretty("No FrameA link exists named '"<<request.Frames[i].FrameALinkName<<"'");
            }
        if(request.Frames[i].FrameBLinkName=="")
            Solution->Frame[i].FrameB = Root;
        else
            try 
            {
                Solution->Frame[i].FrameB = TreeMap.at(request.Frames[i].FrameBLinkName);
            }
            catch(const std::out_of_range& e)
            {
                throw_pretty("No FrameB link exists named '"<<request.Frames[i].FrameBLinkName<<"'");
            }

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

    debugTree.resize(Tree.size()-1);
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
        if(i>0) debugTree[i-1] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica",getRootFrameName()), tf::resolve("exotica", element->Segment.getName()));
        i++;
    }
    Server::sendTransform(debugTree);
    i = 0;
    for(KinematicFrame&  frame : Solution->Frame)
    {
        tf::Transform T;
        tf::transformKDLToTF(frame.TempB, T);
        debugFrames[i*2] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica",getRootFrameName()), tf::resolve("exotica","Frame"+std::to_string(i)+"B"+frame.FrameB->Segment.getName()));
        tf::transformKDLToTF(frame.TempAB, T);
        debugFrames[i*2+1] = tf::StampedTransform(T, ros::Time::now(), tf::resolve("exotica","Frame"+std::to_string(i)+"B"+frame.FrameB->Segment.getName()), tf::resolve("exotica","Frame"+std::to_string(i)+"A"+frame.FrameA->Segment.getName()));
        i++;
    }
    Server::sendTransform(debugFrames);
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
    while(it!=nullptr)
    {
        if(it->IsControlled)
        {
            KDL::Frame SegmentReference;
            if(it->Parent!=nullptr) SegmentReference = it->Parent->Frame;
            J.setColumn(it->ControlId, frame.TempB.M.Inverse()*(SegmentReference.M*it->Segment.twist(TreeState(it->Id), 1.0)).RefPoint(frame.TempA.p-it->Frame.p));

        }
        it = it->Parent;
    }
    it = frame.FrameB;
    while(it!=nullptr)
    {
        if(it->IsControlled)
        {
            KDL::Frame SegmentReference;
            if(it->Parent!=nullptr) SegmentReference = it->Parent->Frame;
            J.setColumn(it->ControlId, J.getColumn(it->ControlId) - (frame.TempB.M.Inverse()*(SegmentReference.M*it->Segment.twist(TreeState(it->Id), 1.0)).RefPoint(frame.TempA.p-it->Frame.p)));
        }
        it = it->Parent;
    }
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
  for (std::shared_ptr<KinematicElement> it : ControlledJoints)
  {
    limits[it->Segment.getJoint().getName()] = it->JointLimits;
  }
  return limits;
}

void KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double> & lower, const std::vector<double> & upper)
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
  if (ControlledBaseType == BASE_TYPE::FLOATING)
  {
    ControlledJoints[0]->JointLimits = {-0.05, 0.05};

    ControlledJoints[1]->JointLimits = {-0.05, 0.05};

    ControlledJoints[2]->JointLimits = {0.875, 1.075};

    ControlledJoints[3]->JointLimits = {-0.087 / 2, 0.087 / 2};

    ControlledJoints[4]->JointLimits = {-0.087 / 2, 0.2617 / 2};

    ControlledJoints[5]->JointLimits = {-M_PI / 8, M_PI / 8};
  }
  else if (ControlledBaseType == BASE_TYPE::PLANAR)
  {
    ControlledJoints[0]->JointLimits = {-10, 10};

    ControlledJoints[1]->JointLimits = {-10, 10};

    ControlledJoints[2]->JointLimits = {-1.57, 1.57};
  }
}

std::string KinematicTree::getRootFrameName()
{
  return Tree[0]->Segment.getName();
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

    for(int i=0; i<ModelJointsNames.size(); i++)
    {
        ret(i) =  TreeState(ModelJointsMap.at(ModelJointsNames[i])->Id);
    }
    return ret;
}

std::map<std::string, double> KinematicTree::getModelStateMap()
{
    std::map<std::string, double> ret;
    for(std::string& jointName : ModelJointsNames)
    {
        ret[jointName] =  TreeState(ModelJointsMap.at(jointName)->Id);
    }
    return ret;
}

void KinematicTree::setModelState(Eigen::VectorXdRefConst x)
{
    if(x.rows()!=ModelJointsNames.size()) throw_pretty("Model state vector has wrong size, expected " << ModelJointsNames.size() << " got " << x.rows());
    for(int i=0; i<ModelJointsNames.size(); i++)
    {
        TreeState(ModelJointsMap.at(ModelJointsNames[i])->Id) = x(i);
    }

    UpdateFK();
    if (Flags & KIN_J) UpdateJ();
    if (Debug) publishFrames();
}

void KinematicTree::setModelState(std::map<std::string, double> x)
{
    for(auto& joint : x)
    {
        try
        {
            TreeState(ModelJointsMap.at(joint.first)->Id) = joint.second;
        }
        catch (const std::out_of_range& e)
        {
            throw_pretty("Robot model does not contain joint '"<<joint.first<<"'");
        }
    }

    UpdateFK();
    if (Flags & KIN_J) UpdateJ();
    if (Debug) publishFrames();
}

Eigen::VectorXd KinematicTree::getControlledState()
{
    Eigen::VectorXd x(NumControlledJoints);
    for(int i=0; i<ControlledJoints.size(); i++)
    {
        x(i) = TreeState(ControlledJoints[i]->Id);
    }
    return x;
}

}
