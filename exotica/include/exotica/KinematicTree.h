/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University of Edinburgh
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

#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Eigen>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <exotica/KinematicElement.h>
#include <exotica/Server.h>
#include <exotica/Tools.h>

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
    KinematicRequestFlags Flags = KinematicRequestFlags::KIN_FK;
    std::vector<KinematicFrameRequest> Frames;  //!< The segments to which the end-effectors are attached
};

struct KinematicFrame
{
    std::weak_ptr<KinematicElement> FrameA;
    KDL::Frame FrameAOffset;
    std::weak_ptr<KinematicElement> FrameB;
    KDL::Frame FrameBOffset;
    KDL::Frame TempAB;
    KDL::Frame TempA;
    KDL::Frame TempB;
};

/**
 * @brief      The KinematicResponse is the container to keep kinematic update
 * data. The corresponding KinematicSolution is created from and indexes into a
 * KinematicResponse.
 */
class KinematicResponse
{
public:
    KinematicResponse();
    KinematicResponse(KinematicRequestFlags Flags, int Size, int N = 0);
    KinematicRequestFlags Flags = KinematicRequestFlags::KIN_FK;
    std::vector<KinematicFrame> Frame;
    Eigen::VectorXd X;
    ArrayFrame Phi;
    ArrayTwist PhiDot;
    ArrayJacobian J;
    ArrayJacobian JDot;
};

/**
 * @brief      The KinematicSolution is created from - and maps into - a KinematicResponse.
 */
class KinematicSolution
{
public:
    KinematicSolution();
    KinematicSolution(int start, int length);
    void Create(std::shared_ptr<KinematicResponse> solution);
    int Start = -1;
    int Length = -1;
    Eigen::Map<Eigen::VectorXd> X{nullptr, 0};
    Eigen::Map<ArrayFrame> Phi{nullptr, 0};
    Eigen::Map<ArrayTwist> PhiDot{nullptr, 0};
    Eigen::Map<ArrayJacobian> J{nullptr, 0};
    Eigen::Map<ArrayJacobian> JDot{nullptr, 0};
};

class KinematicTree : public Uncopyable
{
public:
    KinematicTree();
    virtual ~KinematicTree() = default;
    void Instantiate(std::string JointGroup, robot_model::RobotModelPtr model, const std::string& name);
    std::string getRootFrameName();
    std::string getRootJointName();
    robot_model::RobotModelPtr getRobotModel();
    BASE_TYPE getModelBaseType();
    BASE_TYPE getControlledBaseType();
    std::shared_ptr<KinematicResponse> RequestFrames(const KinematicsRequest& request);
    void Update(Eigen::VectorXdRefConst x);
    void resetJointLimits();
    Eigen::MatrixXd getJointLimits() const { return jointLimits_; }
    void setJointLimitsLower(Eigen::VectorXdRefConst lower_in);
    void setJointLimitsUpper(Eigen::VectorXdRefConst upper_in);
    void setFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double>& lower, const std::vector<double>& upper);
    std::map<std::string, std::vector<double>> getUsedJointLimits();
    int getNumControlledJoints();
    int getNumModelJoints();
    void publishFrames();
    std::vector<std::string> getJointNames()
    {
        return ControlledJointsNames;
    }
    std::vector<std::string> getControlledLinkNames()
    {
        return ControlledLinkNames;
    }

    std::vector<std::string> getModelJointNames()
    {
        return ModelJointsNames;
    }
    std::vector<std::string> getModelLinkNames()
    {
        return ModelLinkNames;
    }

    Eigen::VectorXd getControlledLinkMass();

    KDL::Frame FK(KinematicFrame& frame);
    KDL::Frame FK(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB);
    KDL::Frame FK(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB);
    Eigen::MatrixXd Jacobian(std::shared_ptr<KinematicElement> elementA, const KDL::Frame& offsetA, std::shared_ptr<KinematicElement> elementB, const KDL::Frame& offsetB);
    Eigen::MatrixXd Jacobian(const std::string& elementA, const KDL::Frame& offsetA, const std::string& elementB, const KDL::Frame& offsetB);

    void resetModel();
    std::shared_ptr<KinematicElement> AddElement(const std::string& name, Eigen::Isometry3d& transform, const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& Color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), bool isControlled = false);
    void AddEnvironmentElement(const std::string& name, Eigen::Isometry3d& transform, const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& Color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), bool isControlled = false);
    std::shared_ptr<KinematicElement> AddElement(const std::string& name, Eigen::Isometry3d& transform, const std::string& parent, const std::string& shapeResourcePath, Eigen::Vector3d scale = Eigen::Vector3d::Ones(), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& Color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), bool isControlled = false);
    void UpdateModel();
    void changeParent(const std::string& name, const std::string& parent, const KDL::Frame& pose, bool relative);
    int IsControlled(std::shared_ptr<KinematicElement> Joint);
    int IsControlled(std::string jointName);
    int IsControlledLink(std::string linkName);

    Eigen::VectorXd getModelState();
    std::map<std::string, double> getModelStateMap();
    void setModelState(Eigen::VectorXdRefConst x);
    void setModelState(std::map<std::string, double> x);
    Eigen::VectorXd getControlledState();

    std::vector<std::weak_ptr<KinematicElement>> getTree() { return Tree; }
    std::vector<std::shared_ptr<KinematicElement>> getModelTree() { return ModelTree; }
    std::map<std::string, std::weak_ptr<KinematicElement>> getTreeMap() { return TreeMap; }
    std::map<std::string, std::weak_ptr<KinematicElement>> getCollisionTreeMap() { return CollisionTreeMap; }
    std::map<std::string, std::weak_ptr<KinematicElement>>& getModelJointsMap() { return ModelJointsMap; }
    bool doesLinkWithNameExist(std::string name);  //!< Checks whether a link with this name exists in any of the trees
    bool Debug = false;

    std::map<std::string, shapes::ShapeType> getCollisionObjectTypes();

    // Random state generation
    Eigen::VectorXd getRandomControlledState();

    void setKinematicResponse(std::shared_ptr<KinematicResponse> response_in) { Solution = response_in; }
    std::shared_ptr<KinematicResponse> getKinematicResponse() { return Solution; }
private:
    void BuildTree(const KDL::Tree& RobotKinematics);
    void AddElement(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent);
    void UpdateTree();
    void UpdateFK();
    void UpdateJ();
    void ComputeJ(KinematicFrame& frame, KDL::Jacobian& J);
    void ComputeJdot(KDL::Jacobian& J, KDL::Jacobian& JDot);
    void UpdateJdot();

    // Joint limits
    Eigen::MatrixXd jointLimits_;
    void updateJointLimits();

    // Random state generation
    std::random_device rd;
    std::mt19937 generator_;
    std::vector<std::uniform_real_distribution<double>> random_state_distributions_;

    BASE_TYPE ModelBaseType;
    BASE_TYPE ControlledBaseType = BASE_TYPE::FIXED;
    int NumControlledJoints;  //!< Number of controlled joints in the joint group.
    int NumJoints;            //!< Number of joints of the model (including floating/planar base, passive joints, and uncontrolled joints).
    int StateSize = -1;
    Eigen::VectorXd TreeState;
    robot_model::RobotModelPtr Model;
    std::string root_joint_name_ = "";
    std::vector<std::weak_ptr<KinematicElement>> Tree;
    std::vector<std::shared_ptr<KinematicElement>> ModelTree;
    std::vector<std::shared_ptr<KinematicElement>> EnvironmentTree;
    std::map<std::string, std::weak_ptr<KinematicElement>> TreeMap;
    std::map<std::string, std::weak_ptr<KinematicElement>> CollisionTreeMap;
    std::shared_ptr<KinematicElement> Root;
    std::vector<std::weak_ptr<KinematicElement>> ControlledJoints;
    std::map<std::string, std::weak_ptr<KinematicElement>> ControlledJointsMap;
    std::map<std::string, std::weak_ptr<KinematicElement>> ModelJointsMap;
    std::vector<std::string> ModelJointsNames;
    std::vector<std::string> ControlledJointsNames;
    std::vector<std::string> ModelLinkNames;
    std::vector<std::string> ControlledLinkNames;
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
