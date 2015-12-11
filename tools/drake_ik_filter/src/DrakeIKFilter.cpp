/*
 * DrakeIKFilter.cpp
 *
 *  Created on: 27 Nov 2015
 *      Author: yiming
 */

#include "drake_ik_filter/DrakeIKFilter.h"

namespace exotica
{
  DrakeIKFilter::DrakeIKFilter()
      : model_(NULL)
  {

  }

  DrakeIKFilter::~DrakeIKFilter()
  {
    if (model_) delete model_;
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
  }

  EReturn DrakeIKFilter::initialise(const std::string &urdf)
  {
    if (model_) delete model_;
    model_ = new RigidBodyManipulator(urdf);

    Eigen::Vector2d tspan01;
    tspan01 << 0, 1;
    Eigen::VectorXd quat(4);
    quat << 1, 0, 0, 0;
    WorldQuatConstraint* ptr_left = new WorldQuatConstraint(model_,
        model_->findLinkId("rightFoot"), quat, 0, tspan01);
    WorldQuatConstraint* ptr_right = new WorldQuatConstraint(model_,
        model_->findLinkId("leftFoot"), quat, 0, tspan01);

    constraints_.push_back(ptr_left);
    constraints_.push_back(ptr_right);
    Eigen::Vector2d lb, ub;
    lb << 0.1, 0.1;
    ub << 0.4, 0.4;
    Eigen::Matrix3Xd pt = Eigen::Matrix3Xd::Zero(3, 1);
    Point2PointDistanceConstraint* feet_dist =
        new Point2PointDistanceConstraint(model_,
            model_->findLinkId("rightFoot"), model_->findLinkId("leftFoot"), pt,
            pt, lb, ub);
    constraints_.push_back(feet_dist);

    QuasiStaticConstraint* kc_quasi = new QuasiStaticConstraint(model_);
    kc_quasi->setActive(true);
    kc_quasi->setShrinkFactor(0.2);
    {
      int idx = model_->findLinkId("leftFoot");
      Eigen::Matrix3Xd l_foot_pts(3, 4);
      l_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624, -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
      kc_quasi->addContact(1, &idx, &l_foot_pts);
    }
    {
      int idx = model_->findLinkId("rightFoot");
      Eigen::Matrix3Xd r_foot_pts(3, 4);
      r_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624, -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
      kc_quasi->addContact(1, &idx, &r_foot_pts);
    }
    constraints_.push_back(kc_quasi);

    std::vector<double> q_start = { 0.0, 0.0, 1.025, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.30019663134302466, 1.25, 0.0, 0.78539816339744828,
        1.571, 0.0, 0.0, 0.30019663134302466, -1.25, 0.0, -0.78539816339744828,
        1.571, 0.0, 0.0, 0.0, 0.0, -0.48999999999999999, 1.2050000000000001,
        -0.70999999999999996, 0.0, 0.0, 0.0, -0.48999999999999999,
        1.2050000000000001, -0.70999999999999996, 0.0 };
    reach_start_.resize(q_start.size());
    for (int i = 0; i < q_start.size(); i++)
      reach_start_(i) = q_start[i];
    joints_ =
    { "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw", "torsoYaw", "torsoPitch", "torsoRoll","leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch"};
    std::vector<std::string> other_joints = { "lowerNeckPitch", "neckYaw",
        "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll",
        "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw",
        "rightWristRoll", "rightWristPitch" };
    std::vector<int> other_idx(other_joints.size());
    joints_idx_.resize(joints_.size());
    Eigen::VectorXd other_lb(other_joints.size()), other_ub(
        other_joints.size());
    for (int i = 0; i < model_->num_positions; i++)
    {
      bool other = false;
      for (int j = 0; j < other_joints.size(); j++)
      {
        if (model_->getPositionName(i).compare(other_joints[j]) == 0)
        {
          other_idx[j] = i;
          other_lb(j) = reach_start_[i] - 1e-3;
          other_ub(j) = reach_start_[i] + 1e-3;
          other = true;
          break;
        }
      }
      if (!other)
      {
        for (int j = 0; j < joints_.size(); j++)
        {
          if (model_->getPositionName(i).compare(joints_[j]) == 0)
          {
            joints_idx_[j] = i;
            break;
          }
        }
      }
    }
    PostureConstraint* other_cspace = new PostureConstraint(model_);
    other_cspace->setJointLimits(other_joints.size(), other_idx.data(),
        other_lb, other_ub);
    constraints_.push_back(other_cspace);

//    Eigen::VectorXd eff_lb_(3);
//    Eigen::VectorXd eff_ub_(3);
//    eff_lb_ << -0.1, -0.2, 0.3;
//    eff_ub_ << 0.9, 1, 1.4;
//    Eigen::Vector3d pointInLink;
//    pointInLink << 0.08, 0.07, 0;
//    WorldPositionConstraint* eff_pos_ = new WorldPositionConstraint(model_,
//        model_->findLinkId("leftPalm"), pointInLink, eff_lb_, eff_ub_, tspan01);
//    constraints_.push_back(eff_pos_);

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->num_velocities);
    KinematicsCache<double> cache = model_->doKinematics(reach_start_, v, false,
        false);
    {
      int idx = model_->findLinkId("leftFoot");
      Eigen::VectorXd pos = model_->forwardKin(cache, pt, idx, 0, 0, 0).value();
      WorldPositionConstraint* left_foot_pos_ptr = new WorldPositionConstraint(
          model_, idx, Eigen::Vector3d::Zero(), pos, pos, tspan01);
      constraints_.push_back(left_foot_pos_ptr);
    }
    {
      int idx = model_->findLinkId("rightFoot");
      Eigen::VectorXd pos = model_->forwardKin(cache, pt, idx, 0, 0, 0).value();
      WorldPositionConstraint* right_foot_pos_ptr = new WorldPositionConstraint(
          model_, idx, Eigen::Vector3d::Zero(), pos, pos, tspan01);
      constraints_.push_back(right_foot_pos_ptr);
    }

    ik_options_ = new IKoptions(model_);
    return SUCCESS;
  }

  EReturn DrakeIKFilter::convert(const Eigen::VectorXd &state,
      Eigen::VectorXd &drake, bool fix_joints)
  {
    PostureConstraint* cspace = new PostureConstraint(model_);
    if (fix_joints)
    {
      Eigen::VectorXd c_value(joints_.size());
      for (int i = 0; i < joints_.size(); i++)
        c_value(i) = state(joints_idx_[i]);
      Eigen::VectorXd lb, ub;
      lb = c_value - 1e-3 * Eigen::VectorXd::Ones(joints_.size());
      ub = c_value + 1e-3 * Eigen::VectorXd::Ones(joints_.size());
      cspace->setJointLimits(joints_.size(), joints_idx_.data(), lb, ub);
      constraints_.push_back(cspace);
    }
    std::vector<std::string> infeasible_constraint;
    int info;
    inverseKin(model_, state, state, constraints_.size(), &constraints_[0],
        drake, info, infeasible_constraint, *ik_options_);
    if (fix_joints) constraints_.pop_back();
    delete cspace;
    if (info > 10)
    {
      WARNING_NAMED("DrakeIK filter", "Drake IK INFO = "<<info);
      for (int i = 0; i < infeasible_constraint.size(); i++)
        ROS_INFO_STREAM(
            "infeasible constraint "<<i<<" "<<infeasible_constraint[i]);
    }
    return SUCCESS;
  }
}

