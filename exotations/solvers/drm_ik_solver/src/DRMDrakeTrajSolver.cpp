/*
 * DRMDrakeTrajSolver.cpp
 *
 *  Created on: 12 Nov 2015
 *      Author: yiming
 */

#include "drm_ik_solver/DRMDrakeTrajSolver.h"

REGISTER_MOTIONSOLVER_TYPE("DRMDrakeTrajsolver", exotica::DRMDrakeTrajsolver);

namespace exotica
{
  DRMDrakeTrajsolver::DRMDrakeTrajsolver()
      : traj_client_("/DRM_Trajectory", true), DRMDrakeIKsolver()
  {
  }

  DRMDrakeTrajsolver::~DRMDrakeTrajsolver()
  {
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
    if (other_cspace_) delete other_cspace_;
  }

  EReturn DRMDrakeTrajsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    if (!ok(DRMDrakeIKsolver::initDerived(handle)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    return SUCCESS;
  }
  EReturn DRMDrakeTrajsolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    if (!ok(DRMDrakeIKsolver::specifyProblem(pointer))) return FAILURE;
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
    constraints_.clear();
    Eigen::Vector2d tspan01;
    tspan01 << 0, 1;
    Eigen::VectorXd quat(4);
    quat << 1, 0, 0, 0;
    WorldQuatConstraint* ptr_left = new WorldQuatConstraint(
        prob_->getDrakeModel(), prob_->getDrakeModel()->findLinkId("rightFoot"),
        quat, 0, tspan01);
    WorldQuatConstraint* ptr_right = new WorldQuatConstraint(
        prob_->getDrakeModel(), prob_->getDrakeModel()->findLinkId("leftFoot"),
        quat, 0, tspan01);
    constraints_.push_back(ptr_left);
    constraints_.push_back(ptr_right);

    Eigen::Vector2d lb, ub;
    lb << 0.1, 0.1;
    ub << 0.4, 0.4;
    Eigen::Matrix3Xd pt = Eigen::Matrix3Xd::Zero(3, 1);
    Point2PointDistanceConstraint* feet_dist =
        new Point2PointDistanceConstraint(prob_->getDrakeModel(),
            prob_->getDrakeModel()->findLinkId("rightFoot"),
            prob_->getDrakeModel()->findLinkId("leftFoot"), pt, pt, lb, ub);
    constraints_.push_back(feet_dist);

    QuasiStaticConstraint* kc_quasi = new QuasiStaticConstraint(
        prob_->getDrakeModel());
    kc_quasi->setActive(true);
    kc_quasi->setShrinkFactor(0.05);
    {
      int idx = prob_->getDrakeModel()->findLinkId("leftFoot");
      kc_quasi->addContact(1, &idx, &prob_->magic_.l_foot_pts);
    }
    {
      int idx = prob_->getDrakeModel()->findLinkId("rightFoot");
      kc_quasi->addContact(1, &idx, &prob_->magic_.r_foot_pts);
    }
    constraints_.push_back(kc_quasi);

    //TODO
    //Fix other body parts
    other_joints_ =
    { "lowerNeckPitch", "neckYaw",
      "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll",
      "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw",
      "rightWristRoll", "rightWristPitch"};
    other_joints_idx_.resize(other_joints_.size());
    for (int i = 0; i < other_joints_idx_.size(); i++)
      other_joints_idx_[i] = prob_->joints_map_.at(other_joints_[i]);
    other_cspace_ = new PostureConstraint(prob_->getDrakeModel());
    return SUCCESS;
  }
  EReturn DRMDrakeTrajsolver::setGoalState(const Eigen::VectorXd &goal)
  {
    if (goal.rows() != prob_->getDrakeModel()->num_positions)
    {
      ERROR(
          "No. of joints mismatch, drake model requires "<< prob_->getDrakeModel()->num_positions<<" but "<<goal.rows()<<" provided");
      return FAILURE;
    }
    qT_.resize(prob_->getDrakeModel()->num_positions);
    qT_ = goal;
    return SUCCESS;
  }

  EReturn DRMDrakeTrajsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    if (!traj_client_.waitForServer(ros::Duration(2)))
    {
      WARNING_NAMED(object_name_,
          "Can not connect to dynamic reachability map server");
      return FAILURE;
    }

    if (prob_->getDrakeModel()->num_positions != q0.rows())
    {
      WARNING_NAMED(object_name_,
          "Joints mis match Drake model has "<<prob_->getDrakeModel()->num_positions<<" joints, q0 size = "<<q0.rows());
      return FAILURE;
    }
    dynamic_reachability_map::DRMTrajGoal goal;
    goal.q0.data.resize(drm_drake_joints_map_.size());
    for (int i = 0; i < drm_drake_joints_map_.size(); i++)
      goal.q0.data[i] = q0(drm_drake_joints_map_[i]);
    KDL::Rotation tmp_rot(KDL::Rotation::RPY(q0(3), q0(4), q0(5)));
    tmp_rot.GetQuaternion(goal.q0.data[3], goal.q0.data[4], goal.q0.data[5],
        goal.q0.data[6]);
    goal.qT.data.resize(drm_drake_joints_map_.size());
    for (int i = 0; i < drm_drake_joints_map_.size(); i++)
      goal.qT.data[i] = qT_(drm_drake_joints_map_[i]);
    tmp_rot = KDL::Rotation::RPY(qT_(3), qT_(4), qT_(5));
    tmp_rot.GetQuaternion(goal.qT.data[3], goal.qT.data[4], goal.qT.data[5],
        goal.qT.data[6]);
    goal.base_pose.orientation.w = 1;
    goal.base_pose.position.z = 1.025;
    moveit_msgs::PlanningScene msg;
    prob_->scenes_.begin()->second->getPlanningScene()->getPlanningSceneMsg(
        msg);
    goal.ps = msg;
    if (traj_client_.sendGoalAndWait(goal, ros::Duration(100)).state_
        == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      if (traj_client_.getResult()->succeed)
      {
        dynamic_reachability_map::DRMTrajResultConstPtr result =
            traj_client_.getResult();
        solution.resize(result->solution.size() + 2,
            prob_->getDrakeModel()->num_positions);
        solution.row(0) = q0;
        solution.row(solution.rows() - 1) = qT_;
        for (int i = 0; i < result->solution.size(); i++)
        {
          for (int j = 0; j < prob_->getDrakeModel()->num_positions; j++)
            solution(i + 1, j) = result->solution[i].data[j];
        }
//        for (int i = 0; i < result->solution.size(); i++)
//          solution.row(i) = q0;
//        for (int i = 0; i < result->solution.size(); i++)
//        {
//          for (int j = 0; j < drm_drake_joints_map_.size(); j++)
//            solution(i, drm_drake_joints_map_[j]) = result->solution[i].data[j];
//          tmp_rot = KDL::Rotation::Quaternion(
//              traj_client_.getResult()->solution[i].data[3],
//              traj_client_.getResult()->solution[i].data[4],
//              traj_client_.getResult()->solution[i].data[5],
//              traj_client_.getResult()->solution[i].data[6]);
//          tmp_rot.GetRPY(solution(i, 3), solution(i, 4), solution(i, 5));
//        }
        HIGHLIGHT(
            "DRM planning succeeded, trajectory length "<<result->solution.size());
        info = 0;
//        IKFilter(solution);
        return SUCCESS;
      }
      else
      {
        ERROR("DRM Planning Failed");
        return FAILURE;
      }
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
  }

  EReturn DRMDrakeTrajsolver::IKFilter(Eigen::MatrixXd &solution)
  {
    std::vector<std::string> joints(drm_joints_->strings.size() - 1);
    for (int i = 0; i < 6; i++)
      joints[i] = prob_->getDrakeModel()->getPositionName(i);
    for (int i = 6; i < joints.size(); i++)
      joints[i] = drm_joints_->strings[i + 1];
    for (int i = 0; i < joints.size(); i++)
      ROS_INFO_STREAM("Joint "<<i<<" "<<joints[i]);
    PostureConstraint* c_space = new PostureConstraint(prob_->getDrakeModel());
    Eigen::VectorXd q = solution.row(0);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(
        prob_->getDrakeModel()->num_velocities);
    KinematicsCache<double> cache = prob_->getDrakeModel()->doKinematics(q, v,
        false, false);
    {
      int idx = prob_->getDrakeModel()->findLinkId("leftFoot");
      Eigen::Vector3d pt = Eigen::Vector3d::Zero();
      Eigen::VectorXd pos = prob_->getDrakeModel()->forwardKin(cache, pt, idx,
          0, 0, 0).value();

      WorldPositionConstraint* left_pos = new WorldPositionConstraint(
          prob_->getDrakeModel(), idx, Eigen::Vector3d::Zero(), pos, pos);
      constraints_.push_back(left_pos);
    }
    {
      int idx = prob_->getDrakeModel()->findLinkId("rightFoot");
      Eigen::Vector3d pt = Eigen::Vector3d::Zero();
      Eigen::VectorXd pos = prob_->getDrakeModel()->forwardKin(cache, pt, idx,
          0, 0, 0).value();
      WorldPositionConstraint* right_pos = new WorldPositionConstraint(
          prob_->getDrakeModel(), idx, Eigen::Vector3d::Zero(), pos, pos);
      constraints_.push_back(right_pos);
    }

    std::vector<int> joint_idx(joints.size());
    for (int i = 0; i < joints.size(); i++)
    {
      joint_idx[i] = prob_->joints_map_.at(joints[i]);
    }
    constraints_.push_back(c_space);

    constraints_.push_back(other_cspace_);
    Eigen::VectorXd other_curr(other_joints_.size());
    for (int i = 0; i < other_joints_.size(); i++)
      other_curr(i) = solution(0, other_joints_idx_[i]);
    other_cspace_->setJointLimits(other_joints_.size(),
        other_joints_idx_.data(), other_curr, other_curr);
    std::vector<std::string> infeasible_constraint;
    Eigen::VectorXd tmp_curr(solution.cols()), tmp_last(solution.cols()),
        tmp_sol(solution.cols());
    for (int i = 1; i < solution.rows(); i++)
    {
      tmp_curr = solution.row(i);
      Eigen::VectorXd cspace_curr(joints.size());
      for (int j = 0; j < joints.size(); j++)
        cspace_curr(j) = tmp_curr(prob_->joints_map_.at(joints[j]));
      c_space->setJointLimits(joints.size(), joint_idx.data(), cspace_curr,
          cspace_curr);
      tmp_last = solution.row(i - 1);
      inverseKin(prob_->getDrakeModel(), tmp_curr, tmp_last,
          constraints_.size(), &constraints_[0], tmp_sol, info,
          infeasible_constraint, *ik_options_);
      solution.row(i) = tmp_sol;
    }

//    delete c_space;
    return SUCCESS;
  }
}
