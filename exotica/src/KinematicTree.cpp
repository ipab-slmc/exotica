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

#ifdef KIN_DEBUG_MODE
#include <iostream>
#endif

int exotica::KinematicTree::getNumJoints()
{
  return num_jnts_spec_;
}

exotica::KinematicTree::KinematicTree()
{
#ifdef KIN_DEBUG_MODE
  std::cout << "Default Constructor ... ";
#endif

  //!< Set to default values
  zero_undef_jnts_ = false;

  INFO("Done");
}

exotica::KinematicTree::KinematicTree(const exotica::KinematicTree & rhs)
{
  robot_tree_ = rhs.robot_tree_;
  segment_map_ = rhs.segment_map_;
  used_joints_ = rhs.used_joints_;
  used_joints_segs_ = rhs.used_joints_segs_;
  zero_undef_jnts_ = rhs.zero_undef_jnts_;
  num_jnts_spec_ = rhs.num_jnts_spec_;
  eff_segments_ = rhs.eff_segments_;
  eff_seg_offs_ = rhs.eff_seg_offs_;
  eff_segments_ini_ = rhs.eff_segments_ini_;
  eff_seg_offs_ini_ = rhs.eff_seg_offs_ini_;

  forward_map_ = rhs.forward_map_;
  jacobian_ = rhs.jacobian_;
  com_ = rhs.com_;
}

exotica::KinematicTree & exotica::KinematicTree::operator=(
    const exotica::KinematicTree & rhs)
{
  robot_tree_ = rhs.robot_tree_;
  segment_map_ = rhs.segment_map_;
  used_joints_ = rhs.used_joints_;
  used_joints_segs_ = rhs.used_joints_segs_;
  zero_undef_jnts_ = rhs.zero_undef_jnts_;
  num_jnts_spec_ = rhs.num_jnts_spec_;
  eff_segments_ = rhs.eff_segments_;
  eff_seg_offs_ = rhs.eff_seg_offs_;
  eff_segments_ini_ = rhs.eff_segments_ini_;
  eff_seg_offs_ini_ = rhs.eff_seg_offs_ini_;
  forward_map_ = rhs.forward_map_;
  jacobian_ = rhs.jacobian_;
  com_ = rhs.com_;
  return *this;
}

void exotica::KinematicTree::Instantiate(const KinematicaInitializer& init, robot_model::RobotModelPtr model)
{
    exotica::SolutionForm_t solution;
    solution.root_segment = init.Root.getValue().Segment;
    solution.root_seg_off = init.Root.getValue().Frame;

    if (init.BaseType.getValue() == "floating")
        base_type_ = solution.base_type = init.BaseType;
    else if (init.BaseType.getValue() == "planar")
        base_type_ = solution.base_type = init.BaseType;
    else
        base_type_ = solution.base_type = "fixed";

    controlled_base_ = init.ControlledBase;

    solution.zero_other_joints = init.ZeroOtherJoints;
    solution.joints_update = init.Joints;

    if (solution.joints_update.size() < 1)
    {
        throw_pretty("No update joint is specified");
    }

    if (!model)
    {
      throw_pretty("No robot model provided!");
    }
    else
    {
      model_ = model;
      KDL::Tree temp_tree;
      boost::mutex::scoped_lock(member_lock_);
      if (kdl_parser::treeFromUrdfModel(*model_->getURDF(), temp_tree))
      {
        if(!initialise(temp_tree, solution))
        {
            throw_pretty("Can't initialize Kinematica!");
        }
      }
      else
      {
        throw_pretty("Can't load URDF model!");
      }
    }
}

bool exotica::KinematicTree::initKinematics(tinyxml2::XMLHandle & handle,
    const robot_model::RobotModelPtr model)
{
  INFO("Initialisation from xml");

  exotica::SolutionForm_t solution;

  //!< Checks for compulsaries

  if (!handle.FirstChildElement("Update").ToElement())
  {
    ERROR("Update element not exist");
    return false;
  } //!< We must have the list of joints

//!< Now the solution params:
  solution.root_segment = "";
  base_type_ = solution.base_type = "fixed";
  solution.root_seg_off = KDL::Frame::Identity();
  if (handle.FirstChildElement("Root").ToElement())
  {
    if (!handle.FirstChildElement("Root").ToElement()->Attribute("segment"))
    {
      ERROR("Root element not exist");
      return false;
    }
    solution.root_segment =
        handle.FirstChildElement("Root").ToElement()->Attribute("segment");

    if (handle.FirstChildElement("Root").ToElement()->Attribute("type"))
    {
      std::string base_type =
          handle.FirstChildElement("Root").ToElement()->Attribute("type");
      if (base_type.compare("floating"))
        base_type_ = solution.base_type = base_type;
      else if (base_type.compare("planar"))
        base_type_ = solution.base_type = base_type;
      else
        base_type_ = solution.base_type = "fixed";
    }

    controlled_base_ = true;
    if (handle.FirstChildElement("Root").ToElement()->Attribute(
        "controlled_root"))
    {
      std::string controlled_base =
          handle.FirstChildElement("Root").ToElement()->Attribute(
              "controlled_root");
      if (controlled_base.compare("false") == 0) controlled_base_ = false;
    }

    if (handle.FirstChildElement("Root").FirstChildElement("vector").ToElement())
    {
      Eigen::VectorXd temp_vector;
      if (!xmlGetVector(
          *(handle.FirstChildElement("Root").FirstChildElement("vector").ToElement()),
          temp_vector))
      {
        ERROR("Get root position vector failed");
        return false;
      }
      if (temp_vector.size() != 3)
      {
        ERROR("Root position vector size is invalid");
        return false;
      }
      solution.root_seg_off.p.x(temp_vector(0));
      solution.root_seg_off.p.y(temp_vector(1));
      solution.root_seg_off.p.z(temp_vector(2));
    }

    if (handle.FirstChildElement("Root").FirstChildElement("quaternion").ToElement())
    {
      Eigen::VectorXd temp_vector;
      if (!xmlGetVector(
          *(handle.FirstChildElement("Root").FirstChildElement("quaternion").ToElement()),
          temp_vector))
      {
        ERROR("Get root quaternion failed");
        return false;
      }
      if (temp_vector.size() != 4)
      {
        ERROR("Root quaternion vector size is invalid");
        return false;
      }
      solution.root_seg_off.M = KDL::Rotation::Quaternion(temp_vector(1),
          temp_vector(2), temp_vector(3), temp_vector(0));
    }
  }

  solution.zero_other_joints = true;  //!< By default it is true
  if (handle.FirstChildElement("Update").ToElement()->Attribute("zero_unnamed")) //!< If it exists
  {
    if (handle.FirstChildElement("Update").ToElement()->QueryBoolAttribute(
        "zero_unnamed", &solution.zero_other_joints) != tinyxml2::XML_NO_ERROR)
    {
      ERROR("Update joints are not properly defined");
      return false;
    }  //!< If exists but wrongly defined
  }
  tinyxml2::XMLHandle joint_handle(
      handle.FirstChildElement("Update").FirstChildElement("joint"));
  while (joint_handle.ToElement())
  {
    if (!joint_handle.ToElement()->Attribute("name"))
    {
      ERROR("Update joint names are invalid");
      return false;
    } //!< If no name exists
    solution.joints_update.push_back(
        joint_handle.ToElement()->Attribute("name"));
    joint_handle = joint_handle.NextSiblingElement("joint");
  }
  if (solution.joints_update.size() < 1)
  {
    ERROR("No update joint is specified");
    return false;
  }  //!< If no joints specified

  solution.ignore_unused_segs = true;
  if (handle.FirstChildElement("EndEffector").ToElement())
  {
    if (handle.FirstChildElement("EndEffector").ToElement()->Attribute(
        "ignore_unused"))
    {
      if (handle.FirstChildElement("EndEffector").ToElement()->QueryBoolAttribute(
          "ignore_unused", &solution.ignore_unused_segs)
          != tinyxml2::XML_NO_ERROR)
      {
        ERROR("Invalid end-effector");
        return false;
      }
    }
    tinyxml2::XMLHandle segment_handle(
        handle.FirstChildElement("EndEffector").FirstChildElement("limb"));
    while (segment_handle.ToElement())
    {
      if (!segment_handle.ToElement()->Attribute("segment"))
      {
        ERROR("Invalid end-effector segment");
        return false;
      }
      solution.end_effector_segs.push_back(
          segment_handle.ToElement()->Attribute("segment"));
      KDL::Frame temp_frame = KDL::Frame::Identity(); //!< Initialise to identity
      if (segment_handle.FirstChildElement("vector").ToElement())
      {
        Eigen::VectorXd temp_vector;
        if (!xmlGetVector(
            *(segment_handle.FirstChildElement("vector").ToElement()),
            temp_vector))
        {
          ERROR("Invalid end-effector offset position vector");
          return false;
        }
        if (temp_vector.size() != 3)
        {
          ERROR("Invalid end-effector offset position vector size");
          return false;
        }
        temp_frame.p.x(temp_vector(0));
        temp_frame.p.y(temp_vector(1));
        temp_frame.p.z(temp_vector(2));
      }
      if (segment_handle.FirstChildElement("quaternion").ToElement())
      {
        Eigen::VectorXd temp_vector;
        if (!xmlGetVector(
            *(segment_handle.FirstChildElement("quaternion").ToElement()),
            temp_vector))
        {
          ERROR("Invalid end-effector offset quaternion vector");
          return false;
        }
        if (temp_vector.size() != 4)
        {
          ERROR("Invalid end-effector offset quaternion vector size");
          return false;
        }
        temp_frame.M = KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2),
            temp_vector(3), temp_vector(0));
      }
      solution.end_effector_offs.push_back(temp_frame);
      segment_handle = segment_handle.NextSiblingElement("limb");
    }
  }

  if (model)
    model_ = model;
  else if (base_type_.compare("fixed") != 0)
  {
    ERROR("Kinematica is in non-fixed base mode, but no srdf is provided");
    return false;
  }

  if (model_ == NULL)
  {
    throw_pretty("No robot model provided!");
  }
  else
  {
    KDL::Tree temp_tree;
    boost::mutex::scoped_lock(member_lock_);
    if (kdl_parser::treeFromUrdfModel(*model_->getURDF(), temp_tree))
    {
      return initialise(temp_tree, solution);
    }
    else
    {
      INDICATE_FAILURE
      ;
      return false;
    }
  }
}



bool exotica::KinematicTree::updateEndEffectors(
    const SolutionForm_t & new_end_effectors)
{
//!< Lock for synchronisation
  boost::mutex::scoped_lock(member_lock_);

//!< Clear the needed flag and the end-effector segments vectors
  for (int i = 0; i < robot_tree_.size(); ++i)
  {
    robot_tree_[i].needed = false;
  }
  eff_segments_.clear();
  eff_seg_offs_.clear();

//!< Now do the actual updating
  return setEndEffectors(new_end_effectors);
}

bool exotica::KinematicTree::updateEndEffectorOffsets(
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

bool exotica::KinematicTree::getEndEffectorIndex(std::vector<int> & eff_index)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised()) return false;
  eff_index = eff_segments_;
  return true;
}

std::string exotica::KinematicTree::getBaseType()
{
  return base_type_;
}

bool exotica::KinematicTree::addEndEffector(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
//!< Check if the end-effector is a segment
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
  if (!recurseNeedFlag(segment_map_[name]))
  {
    return false;
  }
  eff_segments_.push_back(segment_map_[name]);
  eff_seg_offs_.push_back(offset);
  forward_map_.resize(3 * eff_segments_.size()); //!< Just position/velocity of end-effector
  jacobian_.resize(3 * eff_segments_.size(), num_jnts_spec_);
  return true;
}

bool exotica::KinematicTree::removeEndEffector(const std::string & name)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
//!< Check if the end-effector is a segment
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
        if (!recurseNeedFlag(eff_segments_[i]))
        {
          return false;
        }
      }
      forward_map_.resize(3 * (N - 1));	//!< Just position/velocity of end-effector
      jacobian_.resize(3 * (N - 1), num_jnts_spec_);
      return true;
    }

  }
  return false;
}

bool exotica::KinematicTree::modifyEndEffector(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
//!< Check if the end-effector is a segment
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

bool exotica::KinematicTree::modifySegment(const std::string & name,
    const KDL::Frame & offset)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
//!< Check if the end-effector is a segment
  std::map<std::string, int>::iterator map_it = segment_map_.find(name);
  if (map_it == segment_map_.end()) return false;
  robot_tree_[map_it->second].offset = offset;
  return true;
}

bool exotica::KinematicTree::updateConfiguration(
    const Eigen::Ref<const Eigen::VectorXd> & joint_configuration)
{
//!< Temporaries
  double jnt_angle;

  //!< Locking
  boost::mutex::scoped_lock(member_lock_);

//!< Checks
  if (!isInitialised())
  {
    ERROR("Scene was not initialized!");
    return false;
  }
  if (!zero_undef_jnts_ && joint_configuration.size() != num_jnts_spec_)
  {
    ERROR(
        "Joint vector size is incorrect!\nExpected "<<joint_configuration.size()<<", found "<<num_jnts_spec_);
    return false;
  }

//!< Start update: start first with the root node...
  jnt_angle =
      (robot_tree_[0].joint_type == JNT_UNUSED) ?
          0 : joint_configuration[robot_tree_[0].joint_index];
  robot_tree_[0].joint_pose = robot_tree_[0].tip_pose
      * robot_tree_[0].segment.pose(jnt_angle).Inverse();
  if (robot_tree_[0].joint_type)	//!< Will be greater than 0
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
    if (robot_tree_[i].needed)	//!< Only proceed if needed
    {
      //!< Temporaries
      KDL::Frame parent_transform;

      //!< Settle Angle
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

      //!< Settle which parent transform to use
      if (robot_tree_[i].from_tip)//If we are coming from the tip of the parent
      {
        parent_transform = robot_tree_[robot_tree_[i].parent].tip_pose;
      }
      else
      {
        parent_transform = robot_tree_[robot_tree_[i].parent].joint_pose;
      }

      //!< Now settle the tip or base
      if (robot_tree_[i].to_tip)	//!< We are moving towards the tip
      {	//!< We generally do not need to concern ourselves with the joint_pose: however might need it for the joint origin computation
        robot_tree_[i].tip_pose = parent_transform
            * robot_tree_[i].segment.pose(jnt_angle) * robot_tree_[i].offset;
        robot_tree_[i].joint_pose = parent_transform * robot_tree_[i].offset;
      }
      else //!< Moving towards the base
      {
        robot_tree_[i].tip_pose = parent_transform * robot_tree_[i].offset;	//!< We cannot be moving towards base from a tip
        robot_tree_[i].joint_pose = parent_transform
            * robot_tree_[i].segment.pose(jnt_angle).Inverse()
            * robot_tree_[i].offset;
      }

      //!< Finally set the joint_origin/axis
      if (robot_tree_[i].joint_type)//!< If we are specifying this joint: if not, it does not make sense to compute it
      {
        if (robot_tree_[i].joint_type == JNT_ROTARY) //!< Only compute axis if rotary
        {
          robot_tree_[i].joint_origin = vectorKdlToEigen(
              robot_tree_[i].joint_pose
                  * robot_tree_[i].segment.getJoint().JointOrigin()); //!< Origin is just transformed into the global frame
        }
        robot_tree_[i].joint_axis = vectorKdlToEigen(
            robot_tree_[i].joint_pose.M
                * robot_tree_[i].segment.getJoint().JointAxis()); //!< For the axes, we do not care about co-ordinates: we just pre-multiply by rotation: this is needed always, for both rotary and prismatic joints
        if (!robot_tree_[i].to_tip)
        {
          robot_tree_[i].joint_axis = -1.0 * robot_tree_[i].joint_axis;
        }
      }
    }
  }
  return true;
}

bool exotica::KinematicTree::setBasePose(const KDL::Frame &pose)
{
  current_base_pose_ = pose;
  return true;
}

bool exotica::KinematicTree::setBaseBounds(const std::vector<double> &bounds)
{
  if (bounds.size() != 6)
  {
    ERROR("Expect base bounds size 6, but received bounds size "<<bounds.size());
    return false;
  }

  if (base_type_.compare("floating") == 0)
  {
    //Here we are just setting the bounds of allowed base movement
    robot_tree_[1].joint_limits_.resize(2);
    robot_tree_[1].joint_limits_[0] = bounds[0];
    robot_tree_[1].joint_limits_[1] = bounds[3];

    robot_tree_[2].joint_limits_.resize(2);
    robot_tree_[2].joint_limits_[0] = bounds[1];
    robot_tree_[2].joint_limits_[1] = bounds[4];

    robot_tree_[3].joint_limits_.resize(2);
    robot_tree_[3].joint_limits_[0] = bounds[2];
    robot_tree_[3].joint_limits_[1] = bounds[5];
  }
  return true;
}

bool exotica::KinematicTree::generateForwardMap()
{
  boost::mutex::scoped_lock(member_lock_);	//!< Lock:
  return computePhi();
}

bool exotica::KinematicTree::generateForwardMap(Eigen::Ref<Eigen::VectorXd> phi)
{
  boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety

  if (computePhi())	//!< If successful:
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

bool exotica::KinematicTree::getPhi(Eigen::Ref<Eigen::VectorXd> phi)
{
  boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety
  if (phi.rows() != forward_map_.rows())
  {
    ERROR(
        "Return vector has wrong size ("<<phi.rows()<<"!="<<forward_map_.rows()<<")!");
    return false;
  }
  phi = forward_map_;
  return true;
}

bool exotica::KinematicTree::generateJacobian()
{
  boost::mutex::scoped_lock(member_lock_); 	//!< Locking
  return computePosJacobian();
}

bool exotica::KinematicTree::getJacobian(Eigen::Ref<Eigen::MatrixXd> jac)
{
  boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety
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

bool exotica::KinematicTree::generateJacobian(
    Eigen::Ref<Eigen::MatrixXd> jacobian)
{
  boost::mutex::scoped_lock(member_lock_); 	//!< Locking

  if (computePosJacobian()) 	//!< If ok...
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

bool exotica::KinematicTree::generateCoM()
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
bool exotica::KinematicTree::getCoMProperties(const std::vector<int>& ids,
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

bool exotica::KinematicTree::getSegment(KDL::Segment & seg, int index)
{
  boost::mutex::scoped_lock(member_lock_);
  if (index < 0 || index >= robot_tree_.size() || !isInitialised())
  {
    return false;	//!< Not initialised, or invalid index
  }
  seg = robot_tree_[index].segment;
  return true;
}

bool exotica::KinematicTree::getSegments(std::vector<KDL::Segment> & segs)
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

bool exotica::KinematicTree::getControlledSegmentsAndJoints(
    std::vector<std::string> & segs, std::vector<std::string> & joints)
{
  boost::mutex::scoped_lock(member_lock_);
  if (!isInitialised())
  {
    return false;
  }
  segs.resize(used_joints_segs_.size());
  segs = used_joints_segs_;
  joints.resize(num_jnts_spec_);
  joints = used_joints_;
  return true;
}

bool exotica::KinematicTree::getInitialEff(std::vector<std::string> & segs,
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

bool exotica::KinematicTree::initialise(const KDL::Tree & temp_tree,
    const exotica::SolutionForm_t & optimisation)
{
    // First clear/reset everything
    robot_tree_.clear();
    segment_map_.clear();
    zero_undef_jnts_ = true;
    num_jnts_spec_ = 0;
    eff_segments_.clear();
    eff_seg_offs_.clear();

    if(buildTree(temp_tree, optimisation.root_segment, joint_map_))
    {
        std::cout << "Kinematica using " << base_type_
            << " base, the robot true root is " << robot_root_.first << " at index "
            << robot_root_.second << std::endl;
        if(setJointLimits())
        {
            if(setJointOrder(optimisation.joints_update,optimisation.zero_other_joints, joint_map_))
            {
                if(setEndEffectors(optimisation))
                {
                    eff_segments_ini_ = optimisation.end_effector_segs;
                    eff_seg_offs_ini_ = optimisation.end_effector_offs;
                    return true;
                }
            }
        }
    }
    // Clear up everything on failure
    robot_tree_.clear();
    segment_map_.clear();
    zero_undef_jnts_ = false;
    num_jnts_spec_ = 0;
    eff_segments_.clear();
    eff_seg_offs_.clear();

    return false;
}

bool exotica::KinematicTree::buildTree(const KDL::Tree & temp_tree,
    std::string root, std::map<std::string, int> & joint_map)
{
  INFO("buildTree Function ... ");

//!< Variable Declarations
  KDL::SegmentMap::const_iterator root_segment; //!< Root segment iterator
  std::string true_root; //!< The urdf root name
  int rubbish; //!< Garbage value since we know root will be at 0

//!< Get the desired segment as root...
  if (root.size() == 0) //!< If no root specified, then we will use the urdf root
  {
    INFO("buildTree Function ... root is of size 0");
    root_segment = temp_tree.getRootSegment();
    INFO("buildTree Function ... root name: ");
  }
  else               //!< We first need to check if this root actually exists...
  {
    INFO("buildTree Function ... root is "<<root);
    KDL::SegmentMap tree_segments = temp_tree.getSegments(); //!< Map of tree segments for checking that desired segment actually exists
    if (tree_segments.find(root) != tree_segments.end()) //!< If it exists...
    {
      root_segment = temp_tree.getSegment(root);
    }
    else
    {
      ERROR("Root "<<root<<" does not exist in the model "<<model_->getName());
      return false; //!< Indicate failure
    }
  }

  robot_root_.first = root_segment->first;

//!< if using floating base
  if (base_type_.compare("fixed") != 0)
  {
    if (model_->getSRDF()->getVirtualJoints().size() != 1)
    {
      ERROR(
          model_->getSRDF()->getVirtualJoints().size()<<" virtual joints are defined, must be set to 1. Can not use floating base");
      return false;
    }
    else
    {
      //	Tricky part, all we need is to create another tree with the floating base and append the robot tree to it
      srdf::Model::VirtualJoint virtual_joint =
          model_->getSRDF()->getVirtualJoints()[0];
      if (virtual_joint.child_link_.compare(root_segment->first) != 0)
      {
        ERROR(
            "Virtual joint has child link "<<virtual_joint.child_link_<<", but robot root is "<<root_segment->first);
        return false;
      }
      std::string world = virtual_joint.parent_frame_;
      std::string world_joint = virtual_joint.name_;
      KDL::Tree base_tree(world);

      if (base_type_.compare("floating") == 0)
      {
        //	Naming based on moveit::core::FloatingJointModel
        //	http://docs.ros.org/indigo/api/moveit_core/html/floating__joint__model_8cpp_source.html
        //	Add translation X
        KDL::Segment transX_seg(world + "/trans_x",
            KDL::Joint(world_joint + "/trans_x", KDL::Joint::TransX));
        if (!base_tree.addSegment(transX_seg, world))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add translation Y
        KDL::Segment transY_seg(world + "/trans_y",
            KDL::Joint(world_joint + "/trans_y", KDL::Joint::TransY));
        if (!base_tree.addSegment(transY_seg, world + "/trans_x"))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add translation Z
        KDL::Segment transZ_seg(world + "/trans_z",
            KDL::Joint(world_joint + "/trans_z", KDL::Joint::TransZ));
        if (!base_tree.addSegment(transZ_seg, world + "/trans_y"))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add rotation X
        KDL::Segment rotX_seg(world + "/rot_x",
            KDL::Joint(world_joint + "/rot_x", KDL::Joint::RotX));
        if (!base_tree.addSegment(rotX_seg, world + "/trans_z"))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add rotation Y
        KDL::Segment rotY_seg(world + "/rot_y",
            KDL::Joint(world_joint + "/rot_y", KDL::Joint::RotY));
        if (!base_tree.addSegment(rotY_seg, world + "/rot_x"))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add rotation Z (which should be named as robot tree's root)
        KDL::Segment rotZ_seg(root_segment->first,
            KDL::Joint(root_segment->first + "/virtual_joint",
                KDL::Joint::RotZ));
        if (!base_tree.addSegment(rotZ_seg, world + "/rot_y"))
        {
          INDICATE_FAILURE
          return false;
        }

        robot_root_.second = 6;
      }
      else if (base_type_.compare("planar") == 0)
      {
        KDL::Segment X_seg(world + "/x",
            KDL::Joint(world_joint + "/x", KDL::Joint::TransX));
        if (!base_tree.addSegment(X_seg, world))
        {
          INDICATE_FAILURE
          return false;
        }
        //	Add translation Y
        KDL::Segment Y_seg(world + "/y",
            KDL::Joint(world_joint + "/y", KDL::Joint::TransY));
        if (!base_tree.addSegment(Y_seg, world + "/x"))
        {
          INDICATE_FAILURE
          return false;
        }
        KDL::Segment rot_seg(root_segment->first,
            KDL::Joint(root_segment->first + "/virtual_joint",
                KDL::Joint::RotZ));
        if (!base_tree.addSegment(rot_seg, world + "/y"))
        {
          INDICATE_FAILURE
          return false;
        }
        robot_root_.second = 3;
      }
      if (base_type_.compare("fixed") == 0)
      {
        return addSegment(temp_tree.getRootSegment(), ROOT, rubbish, true,
            false, world, joint_map);
      }
      else if (base_tree.addTree(temp_tree, root_segment->first))
      {
        return addSegment(base_tree.getRootSegment(), ROOT, rubbish, true,
            false, world, joint_map);
      }
      else
      {
        ERROR(
            "Cant initialise KDL tree for root "<<root<<" with type "<<base_type_);
        return false;
      }

    }

  }
  else
  {
    robot_root_.second = 0;
    true_root = temp_tree.getRootSegment()->second.segment.getName();
    return addSegment(root_segment, ROOT, rubbish, true, false, true_root,joint_map); //!< We do a little trick here to indicate that this is the root node
  }
}

std::map<std::string, int> exotica::KinematicTree::getJointMap()
{
  return joint_map_;
}

std::map<std::string, std::vector<double>> exotica::KinematicTree::getUsedJointLimits()
{
  std::map<std::string, std::vector<double>> limits;
  for (auto & it : joint_map_)
  {
    limits[it.first] = robot_tree_[it.second].joint_limits_;
  }
  return limits;
}

KDL::Frame exotica::KinematicTree::getRobotRootWorldTransform()
{
  KDL::Frame trans = KDL::Frame::Identity();
  if (base_type_.compare("fixed") == 0)
    return trans;
  else
    getPose(robot_root_.second, trans);
  return trans;
}

bool exotica::KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX(
    const std::vector<double> & lower, const std::vector<double> & upper)
{
  if (base_type_.compare("floating") != 0)
  {
    INDICATE_FAILURE
    return false;
  }
  if (lower.size() != 6 || upper.size() != 6)
  {
    INDICATE_FAILURE
    return false;
  }
  for (int i = 0; i < 6; i++)
  {
    robot_tree_[i + 1].joint_limits_[0] = lower[i];
    robot_tree_[i + 1].joint_limits_[1] = upper[i];
  }
  return true;
}
bool exotica::KinematicTree::setJointLimits()
{
  srdf::Model::VirtualJoint virtual_joint =
      model_->getSRDF()->getVirtualJoints()[0];
  std::vector<std::string> vars = model_->getVariableNames();
  for (int i = 0; i < vars.size(); i++)
  {

    if (joint_map_.find(vars[i]) != joint_map_.end())
    {
      int index = joint_map_.at(vars[i]);
      robot_tree_[index].joint_limits_.resize(2);
      robot_tree_[index].joint_limits_[0] =
          model_->getVariableBounds(vars[i]).min_position_;
      robot_tree_[index].joint_limits_[1] =
          model_->getVariableBounds(vars[i]).max_position_;
    }
  }

///	Manually defined floating base limits
///	Should be done more systematically with robot model class
  if (base_type_.compare("floating") == 0)
  {
    robot_tree_[1].joint_limits_.resize(2);
    robot_tree_[1].joint_limits_[0] = -0.05;
    robot_tree_[1].joint_limits_[1] = 0.05;

    robot_tree_[2].joint_limits_.resize(2);
    robot_tree_[2].joint_limits_[0] = -0.05;
    robot_tree_[2].joint_limits_[1] = 0.05;

    robot_tree_[3].joint_limits_.resize(2);
    robot_tree_[3].joint_limits_[0] = 0.875;
    robot_tree_[3].joint_limits_[1] = 1.075;

    robot_tree_[4].joint_limits_.resize(2);
    robot_tree_[4].joint_limits_[0] = -0.087 / 2;
    robot_tree_[4].joint_limits_[1] = 0.087 / 2;

    robot_tree_[5].joint_limits_.resize(2);
    robot_tree_[5].joint_limits_[0] = -0.087 / 2;
    robot_tree_[5].joint_limits_[1] = 0.2617 / 2;

    robot_tree_[6].joint_limits_.resize(2);
    robot_tree_[6].joint_limits_[0] = -M_PI / 8;
    robot_tree_[6].joint_limits_[1] = M_PI / 8;
  }
  else if (base_type_.compare("planar") == 0)
  {
    robot_tree_[1].joint_limits_.resize(2);
    robot_tree_[1].joint_limits_[0] = -10;
    robot_tree_[1].joint_limits_[1] = 10;

    robot_tree_[2].joint_limits_.resize(2);
    robot_tree_[2].joint_limits_[0] = -10;
    robot_tree_[2].joint_limits_[1] = 10;

    robot_tree_[3].joint_limits_.resize(2);
    robot_tree_[3].joint_limits_[0] = -1.57;
    robot_tree_[3].joint_limits_[1] = 1.57;
  }

  return true;
}

bool exotica::KinematicTree::setJointOrder(
    const std::vector<std::string> & joints, bool zero_out,
    const std::map<std::string, int> & joint_map)
{
//!< First do some checks
  if (!zero_out && joints.size() != joint_map.size())
  {
    return false;
  }

//!< Now fill in the joints that will be specified: rest will still be invalidated
  num_jnts_spec_ = joints.size();
  used_joints_.resize(num_jnts_spec_);
  used_joints_segs_.resize(num_jnts_spec_);
  used_joints_ = joints;
  for (int i = 0; i < num_jnts_spec_; i++)
  {
    if (joint_map.find(joints[i]) == joint_map.end()) //!< Guard against possibility of specifying incorrect joints
    {
      ERROR("buildTree Function ...  could not find joint "<<joints[i]);
      return false;
    }
    //!< First check what type of joint it is:
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
      return false;
      break;
    }
    //!< Now set the joint index
    robot_tree_[joint_map.at(joints[i])].joint_index = i;
    used_joints_segs_[i] =
        robot_tree_[joint_map.at(joints[i])].segment.getName();
  }

  return true;
}

bool exotica::KinematicTree::setEndEffectors(
    const SolutionForm_t & optimisation)
{
//!< Variable Declaration
  bool success = true;
  INFO("setEndEffectors Function ...  Entered with offsets of size " << optimisation.end_effector_offs.size());
//!< First do some checks
  if (optimisation.end_effector_offs.size() < 0 //OK if == 0
      && (optimisation.end_effector_segs.size()
          != optimisation.end_effector_offs.size()))
  {
    INDICATE_FAILURE
    ;
    return false;
  }

  INFO("setEndEffectors Function ...  Sizes match up OK!");

//!< Initialise
  robot_tree_[0].tip_pose = (optimisation.root_seg_off).Inverse(); //!< The Root's Tip Pose is the inverse of the given transformation: this will be constant!
  used_joints_segs_ = optimisation.end_effector_segs;

  for (int i = 0; i < optimisation.end_effector_segs.size() && success; i++)
  {
    if (segment_map_.find(optimisation.end_effector_segs[i])
        != segment_map_.end())
    {
      eff_segments_.push_back(segment_map_[optimisation.end_effector_segs[i]]); //!< Push back the index
      if (optimisation.end_effector_offs.size())
      {
        eff_seg_offs_.push_back(optimisation.end_effector_offs[i]);
      } //!< If larger than 0, push back the frame offset
      success = recurseNeedFlag(
          segment_map_[optimisation.end_effector_segs[i]]); //!< Set the needed flag for this and all parents

      INFO("setEndEffectors Function ...  Managed to add End effector " << optimisation.end_effector_segs[i]);
    }
    else
    {
      ERROR(
          "setEndEffectors Function ...  Could not use End effector " << optimisation.end_effector_segs[i]);
      success = false;
    }
  }


  if (success)
  {
    forward_map_.resize(3 * optimisation.end_effector_segs.size());	//!< Just position/velocity of end-effector
    jacobian_.resize(3 * optimisation.end_effector_segs.size(), num_jnts_spec_);

    INFO("setEndEffectors Function ... Created Jacobian of size " << 3*optimisation.end_effector_segs.size() << " x " << num_jnts_spec_);
    if (!optimisation.ignore_unused_segs) //!< If we do not wish to ignore unused chains
    {
      for (int i = 0; i < robot_tree_.size(); i++)
      {
        robot_tree_[i].needed = true;
      } //!< Set all as needed
    }
  }
  else
  {
    INDICATE_FAILURE
    ;
  }

  return success;
}

std::string exotica::KinematicTree::getRootName()
{
  return robot_tree_[0].segment.getName();
}

bool exotica::KinematicTree::modifyRootOffset(KDL::Frame & offset)
{
  if (!isInitialised())
  {
    ERROR("Kinematic tree was not initialized!");
    return false;
  }
  robot_tree_[0].tip_pose = offset.Inverse();
  return true;
}

KDL::Frame exotica::KinematicTree::getRootOffset()
{
  return robot_tree_[0].tip_pose.Inverse();
}

bool exotica::KinematicTree::addSegment(
    KDL::SegmentMap::const_iterator current_segment, int parent, int & current,
    bool from_ptip, bool to_ctip, const std::string & root_name,
    std::map<std::string, int> & joint_map)
{
//!< Variable Declaration
  bool success = true;
  KinematicElement_t current_node;

  INFO("addSegment Function ... with " << current_segment->second.segment.getName() << " parent: " << parent << " flags: " << from_ptip << to_ctip << " root: " << root_name);

//!< First fill in this node
  current_node.parent = parent; //!< Assign the parent
  current_node.from_tip = from_ptip; //!< Indicate whether we reached this through the tip (true) or the base of the parent
  current_node.to_tip = to_ctip; //!< Also check whether we are moving towards the tip or the base
  current_node.segment = current_segment->second.segment; //!< Assign the segment information
  current_node.joint_type = JNT_UNUSED;
  current_node.needed = false; //!< By default not needed
  current = robot_tree_.size(); //!< Set where this node will be stored
  robot_tree_.push_back(current_node); //!< Store

  INFO("addSegment Function ... created node and pushed back on tree at " << robot_tree_.size() - 1);
//!< Update the Segment Map and the Joint Map:
  segment_map_[current_node.segment.getName()] = current;
  joint_map[current_node.segment.getJoint().getName()] = current;

  INFO("addSegment Function ... Indexing Segment and joint maps (" << current_node.segment.getJoint().getName() << ")");

//!< Now comes the tricky part:
  if (to_ctip) //!< We are moving in the forward direction towards the tip (this was a child of the node calling the function)
  {
    INFO("addSegment Function ... Moving to a tip ");
    //!< First Iterate through children
    for (int i = 0; i < current_segment->second.children.size() && success; i++) //!< Iterate through the children if any
    {
      INFO("addSegment Function ... Iterating through children: " << i);
      int child;
      success = addSegment(current_segment->second.children[i], current, child,
          true, true, root_name, joint_map); //!< We are moving from tip towards a tip
      robot_tree_[current].child.push_back(child); //!< Assign Child to this node
    }
    //!< Base Case: If empty, loop will be skipped
  }
  else //!< We are moving towards the base
  {
    INFO("addSegment Function ... Moving to a base ");
    if (from_ptip) //!< This combination (from tip but moving to a base) is impossible, but is used to indicate this is the root node
    {
      INFO("addSegment Function ... Manipulating Root segment ");
      //!< Iterate first through children
      for (int i = 0; i < current_segment->second.children.size() && success;
          i++) //!< Iterate through the children if any
      {
        INFO("addSegment Function ... Iterating through children of root: " << i);
        int child;
        success = addSegment(current_segment->second.children[i], current,
            child, true, true, root_name, joint_map); //!< We are moving from tip towards a tip
        robot_tree_[current].child.push_back(child); //!< Assign Child to this node
      }
      //!< Now handle the parent: only if previously successfull and if this is not the original root node
      if (root_name.compare(current_node.segment.getName()) && success)
      {
        INFO("addSegment Function ... Checking parent of root ");
        int child;
        success = addSegment(current_segment->second.parent, current, child,
            false, false, root_name, joint_map); //!< We are moving from base towards base
        robot_tree_[current].child.push_back(child);	//!< Add as child
      }
    }
    else	//!< I.e. we Are moving from base to a base
    {
      INFO("addSegment Function ... Moving from base to base: ");
      //!< Iterate through children and set them as children of parent rather than current
      for (int i = 0; i < current_segment->second.children.size() && success;
          i++)
      {
        INFO("addSegment Function ... Iterating through children of an inverted segment: " << i);
        int child;
        std::string child_name =
            current_segment->second.children[i]->second.segment.getName();//!< The name of this child
        std::string parent_name = robot_tree_[parent].segment.getName();//!< The name of the parent
        if (parent_name.compare(child_name) == 0)
        {
          continue;
        }									//!< Skip the child who is now parent
        success = addSegment(current_segment->second.children[i], parent, child,
            false, true, root_name, joint_map);
        robot_tree_[parent].child.push_back(child);	//!< Assign child to parent node
      }
      //!< If empty, loop will be skipped
      if (root_name.compare(current_node.segment.getName()) && success)	//!< IF not equal to the root
      {
        INFO("addSegment Function ... Handling parent of inverted segment: ");
        int child;
        success = addSegment(current_segment->second.parent, current, child,
            false, false, root_name, joint_map); //!< Add its parent as its child, but indicate so in the traversal direction
        robot_tree_[current].child.push_back(child);
      }
      //!< Base case if this is indeed the root node in the original urdf
    }
  }
  return success;
}

bool exotica::KinematicTree::recurseNeedFlag(int node)
{
  robot_tree_[node].needed = true;	//!< Indicate that needed
//!< Handle Base Case first:
  if (robot_tree_[node].parent == ROOT)
  {
    return true;
  }
//!< Else, recurse
  else
  {
    return recurseNeedFlag(robot_tree_[node].parent);
  }
}

int exotica::KinematicTree::getEffSize()
{
  return eff_segments_.size();
}

bool exotica::KinematicTree::computePhi()
{
//!< Checks
  if (!isInitialised())
  {
    ERROR("Kinematic tree was not initialized!");
    return false;
  }

  for (int i = 0; i < eff_segments_.size(); i++)
  {
    KDL::Frame end_effector = robot_tree_[eff_segments_[i]].tip_pose;	//!< End effector is w.r.t. tip of segment always
    if (eff_seg_offs_.size())	//!< if Size is greater than 0
    {
      end_effector = end_effector * eff_seg_offs_[i];	//!< Append the respective final transformation
    }
    forward_map_(i * 3) = end_effector.p.x();
    forward_map_(i * 3 + 1) = end_effector.p.y();
    forward_map_(i * 3 + 2) = end_effector.p.z();
  }
  return true;
}

bool exotica::KinematicTree::computePosJacobian()
{
//!< Checks
  if (!isInitialised())
  {
    INDICATE_FAILURE
    ;
    return false;
  }		//!< Ensure that Variables are initialised
  if (!forward_map_.size())
  {
    INDICATE_FAILURE
    ;
    return false;
  }	//!< Ensure that forward_map_ is generated

//!< Compute Jacobian for each end-effector:
  jacobian_.fill(0.0);	//!< Reset everything to 0
  for (int i = 0; i < eff_segments_.size(); i++)//!< Iterate through the end-effectors
  {
    //!< Temporaries
    Eigen::Vector3d end_effector_pos = forward_map_.segment(i * 3, 3);//!< Global End-effector position
    int segment_index = eff_segments_[i];	//!< Current segment index
    while (robot_tree_[segment_index].parent != ROOT)	//!< Repeat until you reach the root joint
    {
      //!< Some tricks: basically, in some cases we may need to execute for both...
      if (robot_tree_[segment_index].to_tip)//!< If in traversing through node we go from base to tip, then we need to consider the effect of its joint
      {
        if (robot_tree_[segment_index].joint_type == JNT_ROTARY)
        {
          Eigen::Vector3d diff_vector = end_effector_pos
              - robot_tree_[segment_index].joint_origin;	//!< Find the p-vector
          jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
              robot_tree_[segment_index].joint_axis.cross(diff_vector);	//!< The Jacobian for this joint
        }
        else if (robot_tree_[segment_index].joint_type == JNT_PRISMATIC)
        {
          jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
              robot_tree_[segment_index].joint_axis; //!< Just the axis
        }
        //!< Else will not be considered
      }
      if (!robot_tree_[segment_index].from_tip) //!< If we are connected to parent from its base, then we (also) need to consider the parents' joint, which also moves us...
      {
        if (robot_tree_[robot_tree_[segment_index].parent].joint_type
            == JNT_ROTARY)
        {
          Eigen::Vector3d diff_vector = end_effector_pos
              - robot_tree_[robot_tree_[segment_index].parent].joint_origin; //!< Find the p-vector (now with its parent joint)
          jacobian_.block(i * 3,
              robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
              robot_tree_[robot_tree_[segment_index].parent].joint_axis.cross(
                  diff_vector); //!< The Jacobian for this joint
        }
        else if (robot_tree_[robot_tree_[segment_index].parent].joint_type
            == JNT_PRISMATIC)
        {
          jacobian_.block(i * 3,
              robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
              robot_tree_[robot_tree_[segment_index].parent].joint_axis;
        }
      }
      segment_index = robot_tree_[segment_index].parent; //!< Move to its parent
    }
  }

//!< If made it this far:
  return true;
}

bool exotica::KinematicTree::getPose(std::string child, std::string parent,
    KDL::Frame & pose)
{
//!< Synchronisation
  boost::mutex::scoped_lock(member_lock_);

//!< Checks
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

//!< Computation
  pose = robot_tree_[segment_map_[parent]].tip_pose.Inverse()
      * robot_tree_[segment_map_[child]].tip_pose;

//!< Return
  return true;
}

bool exotica::KinematicTree::getPose(std::string child, KDL::Frame & pose)
{
//!< Synchronisation
  boost::mutex::scoped_lock(member_lock_);

//!< Checks
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

//!< Computation

  pose = robot_tree_[segment_map_[child]].tip_pose;
//!< Return
  return true;
}

bool exotica::KinematicTree::getPose(int child, int parent, KDL::Frame & pose)
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

bool exotica::KinematicTree::getPose(int child, KDL::Frame & pose)
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

bool exotica::KinematicTree::getSegmentMap(
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

std::string exotica::KinematicTree::getParent(std::string child)
{
  boost::mutex::scoped_lock(member_lock_);

  if (segment_map_.find(child) != segment_map_.end())
  {
    return robot_tree_[robot_tree_[segment_map_[child]].parent].segment.getName();
  }
  else
  {
    return "";  //!< Empty string
  }
}

int exotica::KinematicTree::getParent(int child)
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

std::vector<std::string> exotica::KinematicTree::getChildren(std::string parent)
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
  return children; //!< Which may be an empty array
}

std::vector<int> exotica::KinematicTree::getChildren(int parent)
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

Eigen::Vector3d exotica::vectorKdlToEigen(const KDL::Vector & kdl_vec)
{
  Eigen::Vector3d eigen_vec;

  eigen_vec(0) = kdl_vec.x();
  eigen_vec(1) = kdl_vec.y();
  eigen_vec(2) = kdl_vec.z();

  return eigen_vec;
}

bool exotica::recursivePrint(exotica::KinematicTree & robot, std::string node,
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
    if (i == children.size() - 1) //!< if last element...
    {
      tab.resize(tab.size() - 1);
    }
    success = recursivePrint(robot, children[i], tab);
  }
  return success;
}

bool exotica::xmlGetVector(const tinyxml2::XMLElement & xml_vector,
    Eigen::VectorXd & eigen_vector)
{
//!< Temporaries
  double temp_entry;
  int i = 0;

  if (!xml_vector.GetText())
  {
    eigen_vector = Eigen::VectorXd(); //!< Null matrix again
    INDICATE_FAILURE
    ;
    return false;
  }
  std::istringstream text_parser(xml_vector.GetText());

//!< Initialise looping
  text_parser >> temp_entry;
  while (!(text_parser.fail() || text_parser.bad()))  //!< No commenting!
  {
    eigen_vector.conservativeResize(++i); //!< Allocate storage for this entry (by increasing i)
    eigen_vector(i - 1) = temp_entry;
    text_parser >> temp_entry;
  }
  if (i > 0)
  {
    return true;
  }
  else
  {
    INDICATE_FAILURE
    ;
    return false;
  }
}
