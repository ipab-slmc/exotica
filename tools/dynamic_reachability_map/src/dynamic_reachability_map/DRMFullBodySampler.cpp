/*
 * DRMFullBodySampler.cpp
 *
 *  Created on: 20 Nov 2015
 *      Author: yiming
 */
#include "dynamic_reachability_map/DRMFullBodySampler.h"
#include <moveit/robot_state/conversions.h>

namespace dynamic_reachability_map
{
  DRMFullBodySampler::DRMFullBodySampler(int nt)
      : DRMSampler::DRMSampler(nt), nh_("/FullBodySampler")
  {
  }

  DRMFullBodySampler::~DRMFullBodySampler()
  {
    if (model_) delete model_;
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
  }

  bool DRMFullBodySampler::initialise(const std::string &urdf)
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
    Eigen::VectorXd bb(other_joints.size());
    for (int i = 0; i < model_->num_positions; i++)
    {
      bool other = false;
      for (int j = 0; j < other_joints.size(); j++)
      {
        if (model_->getPositionName(i).compare(other_joints[j]) == 0)
        {
          other_idx[j] = i;
          bb(j) = reach_start_[i];
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
    other_cspace->setJointLimits(other_joints.size(), other_idx.data(), bb, bb);
    constraints_.push_back(other_cspace);

    Eigen::VectorXd eff_lb_(3);
    Eigen::VectorXd eff_ub_(3);
    eff_lb_ << -0.1, -0.2, 0.3;
    eff_ub_ << 0.9, 1, 1.4;
    Eigen::Vector3d pointInLink;
    pointInLink << 0.08, 0.07, 0;
    WorldPositionConstraint* eff_pos_ = new WorldPositionConstraint(model_,
        model_->findLinkId("leftPalm"), pointInLink, eff_lb_, eff_ub_, tspan01);
    constraints_.push_back(eff_pos_);

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->num_velocities);
    KinematicsCache<double> cache = model_->doKinematics(reach_start_, v, false,
        false);
    Eigen::Matrix3Xd pt = Eigen::Matrix3Xd::Zero(3, 1);
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
    state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/SampleState",
        1, true);
    return true;
  }

  unsigned long int DRMFullBodySampler::setToNextState(int id,
      Eigen::Affine3d &effpose, unsigned int &volume_index)
  {
    boost::mutex::scoped_lock(curr_cnt_lock);
    unsigned long int index = curr_cnt_;
    if (index == sample_cnt_) return index + 1;
    curr_cnt_++;

    bool good_sample = false;
    Eigen::VectorXd tmp_sol(model_->num_positions);
    while (!good_sample)
    {
      ps_[id]->getCurrentStateNonConst().setToRandomPositions();
      Eigen::VectorXd tmp_start = reach_start_;
      Eigen::Vector3d tmp_base;
      tmp_base << dRand(-0.087, 0.087), dRand(-0.087, 0.027), dRand(-0.5, 0.05)
          + 1.025;
      tmp_start.segment(0, 3) = tmp_base;

      for (int i = 6; i < joints_.size(); i++)
        tmp_start(joints_idx_[i]) =
            ps_[id]->getCurrentStateNonConst().getVariablePosition(joints_[i]);
      std::vector<std::string> infeasible_constraint;
      int info;
      inverseKin(model_, tmp_start, tmp_start, constraints_.size(),
          &constraints_[0], tmp_sol, info, infeasible_constraint, *ik_options_);
//      ROS_INFO_STREAM("INFO= "<<info);
      if (!(info == 0 || info == 1 || info == 3 || info == 4 || info == 5
          || info == 6)) continue;

      for (int i = 6; i < model_->num_positions; i++)
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            model_->getPositionName(i), (double) tmp_sol(i));
      KDL::Frame base = KDL::Frame(
          KDL::Rotation::RPY(tmp_sol(3), tmp_sol(4), tmp_sol(5)),
          KDL::Vector(tmp_sol(0), tmp_sol(1), tmp_sol(2)));
      std::vector<double> quat(4);
      base.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);

      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/trans_x", base.p.data[0]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/trans_y", base.p.data[1]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/trans_z", base.p.data[2]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/rot_x", quat[0]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/rot_y", quat[1]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/rot_z", quat[2]);
      ps_[id]->getCurrentStateNonConst().setVariablePosition(
          "world_joint/rot_w", quat[3]);

      ps_[id]->getCurrentStateNonConst().update(false);
      effpose = ps_[id]->getCurrentStateNonConst().getGlobalLinkTransform(
          space_->eff_);
      if (!space_->getVolumeIndex(effpose, volume_index)) continue;
//      moveit_msgs::DisplayRobotState state_msg;
//      robot_state::robotStateToRobotStateMsg(ps_[id]->getCurrentState(),
//          state_msg.state);
//      state_pub_.publish(state_msg);
//      ros::spinOnce();
//      ros::Duration(0.05).sleep();
      space_->samples_[index].drake_q = tmp_sol.cast<float>();

      break;
    }
    return index;
  }

  void DRMFullBodySampler::drakeIK(const Eigen::VectorXd &start,
      Eigen::VectorXd &sol, int &info, std::vector<std::string> &infeasible)
  {
    inverseKin(model_, start, reach_start_, constraints_.size(),
        &constraints_[0], sol, info, infeasible, *ik_options_);
  }
}

