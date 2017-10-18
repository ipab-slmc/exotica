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

#ifndef KINEMATIC_TREE_H
#define KINEMATIC_TREE_H

#include <exotica/Tools.h>
#include <moveit/robot_model/robot_model.h>
#include <Eigen/Eigen>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <exotica/Server.h>
#include <tf_conversions/tf_kdl.h>

#include <exotica/KinematicElement.h>

#define ROOT -1  //!< The value of the parent for the root segment

namespace exotica
{
enum BASE_TYPE
{
    FIXED = 0,
    FLOATING = 10,
    PLANAR = 20
};

enum KinematicRequestFlags
{
    KIN_FK = 0,
    KIN_J = 2,
    KIN_FK_VEL = 4,
    KIN_J_DOT = 8
};

inline KinematicRequestFlags operator|(KinematicRequestFlags a, KinematicRequestFlags b)
{
    return static_cast<KinematicRequestFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline KinematicRequestFlags operator&(KinematicRequestFlags a, KinematicRequestFlags b)
{
    return static_cast<KinematicRequestFlags>(static_cast<int>(a) & static_cast<int>(b));
}

class KinematicFrameRequest
{
public:
    KinematicFrameRequest();
    KinematicFrameRequest(std::string frameALinkName, KDL::Frame frameAOffset = KDL::Frame(), std::string frameBLinkName = "", KDL::Frame frameBOffset = KDL::Frame());
    std::string FrameALinkName;
    KDL::Frame FrameAOffset;
    std::string FrameBLinkName;
    KDL::Frame FrameBOffset;
};

class KinematicsRequest
{
public:
    KinematicsRequest();
    KinematicRequestFlags Flags;
    std::vector<KinematicFrameRequest> Frames;  //!< The segments to which the end-effectors are attached
};

struct KinematicFrame
{
    std::shared_ptr<KinematicElement> FrameA;
    KDL::Frame FrameAOffset;
    std::shared_ptr<KinematicElement> FrameB;
    KDL::Frame FrameBOffset;
    KDL::Frame TempAB;
    KDL::Frame TempA;
    KDL::Frame TempB;
};

class KinematicResponse
{
public:
    KinematicResponse();
    KinematicResponse(KinematicRequestFlags Flags, int Size, int N = 0);
    KinematicRequestFlags Flags;
    std::vector<KinematicFrame> Frame;
    ArrayFrame Phi;
    ArrayTwist PhiDot;
    ArrayJacobian J;
    ArrayJacobian JDot;
};

class KinematicSolution
{
public:
    KinematicSolution();
    KinematicSolution(int start, int length);
    void Create(std::shared_ptr<KinematicResponse> solution);
    int Start;
    int Length;
    Eigen::Map<ArrayFrame> Phi;
    Eigen::Map<ArrayTwist> PhiDot;
    Eigen::Map<ArrayJacobian> J;
    Eigen::Map<ArrayJacobian> JDot;
};

class KinematicTree : public Uncopyable
{
public:
    KinematicTree();
    ~KinematicTree() {}
    void Instantiate(std::string JointGroup, robot_model::RobotModelPtr model, const std::string& name);
    std::string getRootFrameName();
    std::string getRootJointName();
    BASE_TYPE getModelBaseType();
    BASE_TYPE getControlledBaseType();
    std::shared_ptr<KinematicResponse> RequestFrames(const KinematicsRequest& request);
    void Update(Eigen::VectorXdRefConst x);
    void setJointLimits();
    void setFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double>& lower, const std::vector<double>& upper);
    std::map<std::string, std::vector<double>> getUsedJointLimits();
    int getEffSize();
    int getNumJoints(); // to be renamed
    int getNumModelJoints();
    void publishFrames();
    Eigen::MatrixXd getJointLimits();
    std::vector<std::string> getJointNames()
    {
        return ControlledJointsNames;
    }

    std::vector<std::string> getModelJointNames()
    {
        return ModelJointsNames;
    }

    KDL::Frame FK(KinematicFrame& frame);
    KDL::Frame FK(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB);
    KDL::Frame FK(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB);
    Eigen::MatrixXd Jacobian(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB);
    Eigen::MatrixXd Jacobian(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB);

    void resetModel();
    std::shared_ptr<KinematicElement> AddElement(const std::string& name, Eigen::Affine3d& transform, const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero());
    void UpdateModel();
    void changeParent(const std::string& name, const std::string& parent, const KDL::Frame& pose, bool relative);

    Eigen::VectorXd getModelState();
    std::map<std::string, double> getModelStateMap();
    void setModelState(Eigen::VectorXdRefConst x);
    void setModelState(std::map<std::string, double> x);
    Eigen::VectorXd getControlledState();

    std::vector<std::shared_ptr<KinematicElement>> getTree() { return Tree; }
    std::map<std::string, std::shared_ptr<KinematicElement>> getCollisionTreeMap() { return CollisionTreeMap; }
    std::map<std::string, std::shared_ptr<KinematicElement>> getTreeMap() { return TreeMap; }
    bool Debug;

private:
    void BuildTree(const KDL::Tree& RobotKinematics);
    void AddElement(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent);
    int IsControlled(std::shared_ptr<KinematicElement> Joint);
    void UpdateTree();
    void UpdateFK();
    void UpdateJ();
    void ComputeJ(const KinematicFrame& frame, KDL::Jacobian& J);

    BASE_TYPE ModelBaseType;
    BASE_TYPE ControlledBaseType = BASE_TYPE::FIXED;
    int NumControlledJoints;  //!< Number of controlled joints in the joint group.
    int NumJoints;            //!< Number of joints of the model (including floating/planar base, passive joints, and uncontrolled joints).
    int StateSize;
    Eigen::VectorXd TreeState;
    robot_model::RobotModelPtr Model;
    std::vector<std::shared_ptr<KinematicElement>> Tree;
    std::vector<std::shared_ptr<KinematicElement>> ModelTree;
    std::map<std::string, std::shared_ptr<KinematicElement>> TreeMap;
    std::map<std::string, std::shared_ptr<KinematicElement>> CollisionTreeMap;
    std::shared_ptr<KinematicElement> Root;
    std::vector<std::shared_ptr<KinematicElement>> ControlledJoints;
    std::map<std::string, std::shared_ptr<KinematicElement>> ControlledJointsMap;
    std::map<std::string, std::shared_ptr<KinematicElement>> ModelJointsMap;
    std::vector<std::string> ModelJointsNames;
    std::vector<std::string> ControlledJointsNames;
    std::shared_ptr<KinematicResponse> Solution;
    KinematicRequestFlags Flags;

    std::vector<tf::StampedTransform> debugTree;
    std::vector<tf::StampedTransform> debugFrames;
    ros::Publisher shapes_pub_;
    bool debugSceneChanged;
    std::string name_;
};
}
#endif
