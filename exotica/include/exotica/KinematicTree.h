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

#include <boost/thread/mutex.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <Eigen/Eigen>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <exotica/Tools.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#define ROOT  -1     //!< The value of the parent for the root segment

namespace exotica
{

    enum BASE_TYPE
    {
      FIXED = 0, FLOATING = 10, PLANAR = 20
    };

  /**
   * \brief Defines the two types of supported joints
   */
  enum JointType
  {
    JNT_UNUSED = 0,  //!< Undefined (will not be used)
    JNT_PRISMATIC = 1,  //!< Prismatic joint
    JNT_ROTARY = 2   //!< Rotary Joint
  };

  enum KinematicRequestFlags
  {
      KIN_FK = 0,
      KIN_J = 2,
      KIN_FK_VEL = 4,
      KIN_J_DOT = 8
  };

  inline KinematicRequestFlags operator|(KinematicRequestFlags a, KinematicRequestFlags b)
  {return static_cast<KinematicRequestFlags>(static_cast<int>(a) | static_cast<int>(b));}

  inline KinematicRequestFlags operator&(KinematicRequestFlags a, KinematicRequestFlags b)
  {return static_cast<KinematicRequestFlags>(static_cast<int>(a) & static_cast<int>(b));}

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
      std::vector<KinematicFrameRequest> Frames; //!< The segments to which the end-effectors are attached
  };

  class KinematicElement
  {
  public:
      KinematicElement(int id, std::shared_ptr<KinematicElement> parent, KDL::Segment segment);
      int Id;
      int ControlId;
      bool IsControlled;
      std::shared_ptr<KinematicElement> Parent;
      std::vector<std::shared_ptr<KinematicElement>> Children;
      KDL::Segment Segment;
      KDL::Frame Frame;
      std::vector<double> JointLimits;
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
      KinematicResponse(KinematicRequestFlags Flags, int Size, int N=0);
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

  struct KinematicElement_t
  {
      KDL::Segment segment;  		//!< The actual segment
      bool needed;//!< Flag to check whether actually needed or not for the FK Computation
      int parent;   		//!< The parent link (index)
      bool from_tip; //!< Whether it is connected to the tip of its parent (true) or to its base
      bool to_tip;//!< Indicates whether we are moving towards the tip (true) or not
      JointType joint_type;   //!< Type of joint
      int joint_index; //!< The Joint connecting this limb to its ORIGINAL parent (mapping into the configuration array), if used
      KDL::Frame tip_pose;     //!< The latest update of the pose w.r.t root
      KDL::Frame joint_pose; //!< The pose of the base of the joint at the base of this segment (N.B. THIS IS THE POSE OF THE BASE OF THE FULL SEGMENT, from base of joint)
      Eigen::Vector3d joint_origin; //!< The Joint origin (actual position of joint-axis, in global frame)
      Eigen::Vector3d joint_axis;	//!< The Joint Axis of rotation (in the global frame)...
      std::vector<int> child;    		//!< Any child links
      Eigen::Vector3d com;	//!< Centre of mass
      std::vector<double> joint_limits_;
      KDL::Frame offset;
  };

    class KinematicTree : public Uncopyable
    {
    public:
            KinematicTree();
            ~KinematicTree() {}
            void Instantiate(std::string JointGroup, robot_model::RobotModelPtr model);
            std::string getRootName();
            BASE_TYPE getBaseType();
            std::shared_ptr<KinematicResponse> RequestFrames(const KinematicsRequest& request);
            void Update(Eigen::VectorXdRefConst x);
            void setJointLimits();
            void setFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double> & lower, const std::vector<double> & upper);
            std::map<std::string, std::vector<double>> getUsedJointLimits();
            int getEffSize();
            int getNumJoints();
            void publishFrames();
            Eigen::MatrixXd getJointLimits();
            std::vector<std::string> getJointNames()
            {
              return ControlledJointsNames;
            }

            std::vector<std::shared_ptr<KinematicElement>> getTree() {return Tree;}
            std::map<std::string, std::shared_ptr<KinematicElement>> getTreeMap() {return TreeMap;}
            bool Debug;

    private:
            void BuildTree(const KDL::Tree & RobotKinematics);
            void AddElement(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent);
            int IsControlled(std::shared_ptr<KinematicElement> Joint);
            void UpdateTree(Eigen::VectorXdRefConst x);
            void UpdateFK();
            void UpdateJ();
            void ComputeJ(const KinematicFrame& frame, KDL::Jacobian& J);

            BASE_TYPE BaseType;
            int NumControlledJoints; //!< Number of controlled joints in the joint group.
            int NumJoints; //!< Number of joints of the robot (including floating/plannar base, passive joints and uncontrolled joints).
            int StateSize;
            Eigen::VectorXd TreeState;
            robot_model::RobotModelPtr Model;
            std::vector<std::shared_ptr<KinematicElement>> Tree;
            std::map<std::string, std::shared_ptr<KinematicElement>> TreeMap;
            std::shared_ptr<KinematicElement> Root;
            std::vector<std::shared_ptr<KinematicElement>> ControlledJoints;
            std::map<std::string, std::shared_ptr<KinematicElement>> ControlledJointsMap;
            std::vector<std::string> ModleJointsNames;
            std::vector<std::string> ControlledJointsNames;
            std::shared_ptr<KinematicResponse> Solution;
            KinematicRequestFlags Flags;

            tf::TransformBroadcaster debugTF;
            std::vector<tf::StampedTransform> debugTree;
            std::vector<tf::StampedTransform> debugFrames;
    };

}
#endif
