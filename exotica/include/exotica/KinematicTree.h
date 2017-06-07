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

      /**
       * \brief Constructor: creates a kinematic tree from a urdf file (uses kdl parser internally)
       * @param urdf_param     Name of the urdf ROS parameter
       * @param optimisation  Optimisation Parameters
       */
      KinematicTree(void);                      //!< Default Constructor

      void Instantiate(std::string JointGroup, robot_model::RobotModelPtr model);

      ~KinematicTree()
      {
      }
      ;

      /**
       * \brief Provides dynamic updating of the end-effector list
       * @param new_end_effectors A solution form type: you only need to set the ignore_unused_segs flag, the end_effector_segs and the end_effector_offs
       * @return                  True if successful, false otherwise
       */
      void updateEndEffectors(const KinematicsRequest & new_end_effectors);

      /**
       * \brief	Update end-effector offset
       * @param	index	End-effector index
       * @param	offset	New offset
       */
      bool updateEndEffectorOffsets(const std::vector<int> & index,
          const std::vector<KDL::Frame> & offset);

      /**
       * \brief	Add new end-effector
       * @param	name	End-effector segment name
       * @param	offset	End-effector offset
       * @return	True if succeeded, false otherwise
       */
      bool addEndEffector(const std::string & name, const KDL::Frame & offset);

      /**
       * \brief	Remove end-effector
       * @param	name	End-effector segment name
       * @return	True if succeeded, false otherwise
       */
      bool removeEndEffector(const std::string & name);

      /**
       * \brief	Modify end-effector
       * @param	name	End-effector segment name
       * @param	offset	End-effector offset
       * @return	True if succeeded, false otherwise
       */
      bool modifyEndEffector(const std::string & name,
          const KDL::Frame & offset);

      /**
       * \brief	Modify segment tip pose
       * @param	name	Segment name
       * @param	offset	Segment offset
       * @return	True if succeeded, false otherwise
       */
      bool modifySegment(const std::string & name,
          const KDL::Frame & offset);

      /**
       * \brief Update the internal position of joints pose of each segment : THREAD-SAFE
       * @param joint_configuration The value for each joint in the order specified by joint ordering.
       * @return                    True if successful, False if not
       */
      bool updateConfiguration(
          const Eigen::Ref<const Eigen::VectorXd> & joint_configuration);

      /**
       * \brief Generate the phi vector (forward kinematics for the specified end-effectors) : THREAD-SAFE
       * @param phi	The Forward Kinematics Vector
       * @return	True if successful, false otherwise
       */
      bool generateForwardMap();
      bool generateForwardMap(Eigen::Ref<Eigen::VectorXd> phi);

      /**
       * \brief Generate the Jacobian: assumes that the forward map is updated
       * @param jacobian The Jacobian Matrix
       * @return 		 True if successful, false otherwise
       */
      bool generateJacobian();
      bool generateJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian);

      /**
       * \brief	Generate the centre of mass position
       * @param	com	Centre of mass
       * @return	True if succeeded, false otherwise
       */
      bool generateCoM();
      bool generateCoM(Eigen::Vector3d & com);
      /**
       * \brief Returns the Jacobian that was computed : THREAD-SAFE
       * @param jacobian  Reference to the Jacobian to modify
       * @return          True if successful, false otherwise
       */
      bool getJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian);

      /**
       * \brief Returns the Forward Kinematics Map that was computed : THREAD-SAFE
       * @param phi Reference to the FK Map to modify
       * @return    True if successful, false otherwise
       */
      bool getPhi(Eigen::Ref<Eigen::VectorXd> phi);

      /**
       * \brief Computes and returns the transformation between a child and a parent link: THREAD-SAFE
       * @param child   Name of the child link/Index of the child link
       * @param parent  Name of the parent link/Index of the parent link: root frame (not root segment) if lacking
       * @param pose    The frame transformation
       * @return        True if no error, false otherwise
       */
      bool getPose(std::string child, std::string parent, KDL::Frame & pose);
      bool getPose(std::string child, KDL::Frame & pose);
      bool getPose(int child, int parent, KDL::Frame & pose);
      bool getPose(int child, KDL::Frame & pose);

      /**
       * \brief Returns the mapping between segment names and their indexes
       * @param segment_map Reference to the map for storing the segment mapping
       * @return            True if success, false otherwise
       */
      bool getSegmentMap(std::map<std::string, int> & segment_map);

      /**
       * \brief	Get the segments
       * @param	seg		KDL Segment
       * @param	segs	KDL Segments for all links
       * @param	index	Index in the tree
       * @return	True if succeeded, false otherwise
       */
      bool getSegment(KDL::Segment & seg, int index);
      bool getSegments(std::vector<KDL::Segment> & segs);

      /**
       * \brief	Get the controlled segments and joints names
       * @param	segs	Segment names
       * @param	joints	Joint names
       * @return	True if succeeded, false otherwise
       */
      bool getControlledSegmentsAndJoints(std::vector<std::string> & segs,
          std::vector<std::string> & joints);
      /**
       * \brief	Get centre of mass properties
       * @param	seg		Segment names
       * @param	mass	Mass of each link
       * @param	cog		Centre of gravity of each link
       * @param	tip_pose	Tip pose of each link
       * @param	base_pose	Base pose of each link
       */
      bool getCoMProperties(const std::vector<int>& ids,
          std::vector<std::string> & segs, Eigen::VectorXd & mass,
          std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
          std::vector<KDL::Frame> & base_pose);

      /**
       * \brief Allows traversing the tree structure by getting the parent
       * @param child   The Name of the child/Index into the array
       * @return        The Parent (either string or index depending on call)
       */
      std::string getParent(std::string child);
      int getParent(int child);

      /**
       * \brief Allows traversing the tree structure by getting the children
       * @param parent  The Name of the parent/Index into the array
       * @return        The vector of children (either string or int index)
       */
      std::vector<std::string> getChildren(std::string parent);
      std::vector<int> getChildren(int parent);

      /**
       * \brief Returns the number of end-effectors.
       * @return Number of end-effectors.
       */

      bool getInitialEff(std::vector<std::string> & segs, std::vector<KDL::Frame> & offsets);
      bool modifyRootOffset(KDL::Frame & offset);
      KDL::Frame getRootOffset();
      std::string getRootName();

      bool getEndEffectorIndex(std::vector<int> & eff_index);

      BASE_TYPE getBaseType();
      std::map<std::string, int> getJointMap();
      std::map<std::string, std::vector<double>> getUsedJointLimits();

      KDL::Frame getRobotRootWorldTransform();
      bool setBasePose(const KDL::Frame &pose);

      //private:
      /****************** Class members ********************/

      /**
       * \brief Robot Tree Structure
       */
      std::vector<KinematicElement_t> robot_tree_;    //!< The tree structure
      std::map<std::string, int> segment_map_; //!< Mapping from segment names to positions in the vector robot_tree_
      std::map<std::string, int> joint_map_;
      std::pair<std::string, int> robot_root_;

      std::vector<std::string> used_joints_;
      std::vector<std::string> used_joints_segs_;
      /**
       * \brief Solver Constants (for optimisation)
       */
      bool zero_undef_jnts_; //!< Indicates whether we wish to zero-out undefined joints.

      BASE_TYPE base_type_;
      bool controlled_base_;
      KDL::Frame current_base_pose_;
      std::vector<int> eff_segments_; //!< The End-Effector segments (for Jacobian Computation)
      std::vector<KDL::Frame> eff_seg_offs_; //!< Offsets from the end effector segments (if required)
      std::vector<std::string> eff_segments_ini_;
      std::vector<KDL::Frame> eff_seg_offs_ini_;

      /**
       *  \brief Solution Storage
       */
      Eigen::VectorXd forward_map_;	//!< The forward mapping
      Eigen::MatrixXd jacobian_;		//!< The Jacobian
      Eigen::Vector3d com_;	//!< The centre of mass

      /**
       * \brief Synchronisation and Thread-Safety
       */
      boost::mutex member_lock_;   //!< Member lock (synchronisation)

      /*************** Private Class Methods *****************/

      /**
       * \brief Builds a robot tree from the kdl tree : NOT THREAD-SAFE
       * @param temp_tree  The KDL Tree
       * @param root       Name of the root segment
       * @param joint_map  The Mapping from the joint to the index for the associated segment
       * @return           True if ok, false if not
       */
      void buildTree(const KDL::Tree & temp_tree, std::map<std::string, int> & joint_map);



      /**
       * \brief Set the Joint ordering we will use : NOT THREAD-SAFE
       * @param joints    The list of joints (in order) that will be used
       * @param zero_out  Indicates whether we will choose to zero out undefined joints
       * @param joint_map Mapping from joint names to indexes of the correspdonding segment
       * @return          True if success, false otherwise
       */
      void setJointOrder(const std::vector<std::string> & joints, bool zero_out,
          const std::map<std::string, int> & joint_map);

      /**
       * \brief Recursive Function which modifies the robot_tree_ and the segment_map_ : NOT THREAD-SAFE
       * TODO
       * @return        True if successful, false otherwise
       */
      void addSegment(KDL::SegmentMap::const_iterator current_segment,
          int parent, int & current, bool from_ptip, bool to_ctip,
          const std::string & root_name,
          std::map<std::string, int> & joint_map);

      /**
       * \brief Recursive function for propagating chains from end-effector to root.
       * @param	node Index into the next node to process.
       * @return	 True if successful, false otherwise
       */
      void recurseNeedFlag(int node);

      /**
       * \brief Checks that everything is ok : NOT THREAD-SAFE
       * @return	Currently returns the state of the robot_tree_..
       */
      inline bool isInitialised()
      {
        return robot_tree_.size();
      }
      ;

      /**
       * \brief Private function for computing Forward Mapping : NOT THREAD-SAFE
       * @return	True if successful, false otherwise
       */
      bool computePhi();

      /**
       * \brief Computes the link-specific Positional Jacobian : NOT THREAD-SAFE
       * @return		True if successful, false otherwise
       */
      bool computePosJacobian();




public:
        std::shared_ptr<KinematicResponse> RequestFrames(const KinematicsRequest& request);
        void Update(Eigen::VectorXdRefConst x);
        void setJointLimits();
        void setBaseBounds(const std::vector<double> &bounds);
        void setFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double> & lower, const std::vector<double> & upper);
        int getEffSize();
        int getNumJoints();
        Eigen::MatrixXd getJointLimits();
        std::vector<std::string> & getJointNames()
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
    };

  /**
   * \brief Helper Functions
   */

  Eigen::Vector3d vectorKdlToEigen(const KDL::Vector & kdl_vec);
  bool recursivePrint(exotica::KinematicTree & robot, std::string node,
      std::string tab);
}
#endif
