/*
 * DRMDrakeIK.cpp
 *
 *  Created on: 13 Oct 2015
 *      Author: yiming
 */

#include "drm_ik_solver/DRMDrakeIKSolver.h"

REGISTER_MOTIONSOLVER_TYPE("DRMDrakeIKsolver", exotica::DRMDrakeIKsolver);
ros::Publisher tmp_pub;
namespace exotica
{
  DRMDrakeIKsolver::DRMDrakeIKsolver():drm_client_(
      "/DRM_IK", true)
  {
    DrakeIKsolver();
  }

  DRMDrakeIKsolver::~DRMDrakeIKsolver()
  {

  }

  EReturn DRMDrakeIKsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    if (!ok(DrakeIKsolver::initDerived(handle)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (!server_->hasParam(server_->getName() + "/DRMJoints"))
    {
      ERROR("DRMJoints needs to be specified as server parameter");
      return FAILURE;
    }
    if (!ok(server_->getParam(server_->getName() + "/DRMJoints", drm_joints_)))
    {
      ERROR("Get DRMJoints failed");
      return FAILURE;
    }
    tmp_pub = server_->advertise<geometry_msgs::PoseStamped>("/CollisionPoint",
        1);
    return SUCCESS;
  }
  EReturn DRMDrakeIKsolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    DrakeIKsolver::specifyProblem(pointer);
    prob_ = boost::static_pointer_cast<DRMDrakeIKProblem>(pointer);
    drm_drake_joints_map_.resize(drm_joints_->strings.size());
    drm_drake_joints_map_[0] = 0;
    drm_drake_joints_map_[1] = 1;
    drm_drake_joints_map_[2] = 2;
    drm_drake_joints_map_[3] = 3;
    drm_drake_joints_map_[4] = 4;
    drm_drake_joints_map_[5] = 5;
    drm_drake_joints_map_[6] = 5;
    for (int i = 6; i < prob_->getDrakeModel()->num_positions; i++)
    {
      for (int j = 7; j < drm_joints_->strings.size(); j++)
      {
        if (prob_->getDrakeModel()->getPositionName(i).compare(
            drm_joints_->strings[j]) == 0) drm_drake_joints_map_[j] = i;
      }
    }
    drm_ps_joints_map_.resize(drm_joints_->strings.size());
    for (int i = 0; i < drm_joints_->strings.size(); i++)
    {
      drm_ps_joints_map_[i] =
          prob_->getScenes().begin()->second->getPlanningScene()->getRobotModel()->getVariableIndex(
              drm_joints_->strings[i]);
    }
    return SUCCESS;
  }
  EReturn DRMDrakeIKsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    if (!drm_client_.waitForServer(ros::Duration(2)))
    {
      WARNING_NAMED(object_name_,
          "Can not connect to dynamic reachability map server");
    }
    if (prob_->getDrakeModel()->num_positions != q0.rows())
    {
      WARNING(
          "Joints mis match Drake model has "<<prob_->getDrakeModel()->num_positions<<" joints, q0 size = "<<q0.rows());
    }
    dynamic_reachability_map::DRMGoal goal;
    moveit_msgs::PlanningScene msg;
    prob_->scenes_.begin()->second->getPlanningScene()->getPlanningSceneMsg(
        msg);
    goal.ps = msg;
    Eigen::VectorXd qs = q0, q_sol(q0.rows());
    goal.q0.data.resize(drm_drake_joints_map_.size());
    for (int i = 0; i < drm_drake_joints_map_.size(); i++)
      goal.q0.data[i] = q0(drm_drake_joints_map_[i]);
    KDL::Rotation tmp_rot(KDL::Rotation::RPY(q0(3), q0(4), q0(5)));
    tmp_rot.GetQuaternion(goal.q0.data[3], goal.q0.data[4], goal.q0.data[5],
        goal.q0.data[6]);
    goal.goal_pose.position.x = prob_->eff_goal_pose.p[0];
    goal.goal_pose.position.y = prob_->eff_goal_pose.p[1];
    goal.goal_pose.position.z = prob_->eff_goal_pose.p[2];
    prob_->eff_goal_pose.M.GetQuaternion(goal.goal_pose.orientation.x,
        goal.goal_pose.orientation.y, goal.goal_pose.orientation.z,
        goal.goal_pose.orientation.w);
    goal.base_pose.orientation.w = 1;
    goal.base_pose.position.z = 1.025;
    std::map<std::string, bool> obs_map;
    for (int i = 0; i < msg.world.collision_objects.size(); i++)
      obs_map[msg.world.collision_objects[i].id] = true;
    int cnt = 0;
    bool succeeded = false;
    ros::Time start = ros::Time::now();
    while (true)
    {
      if (drm_client_.sendGoalAndWait(goal, ros::Duration(2)).state_
          == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if (drm_client_.getResult()->succeed)
        {
          PostureConstraint* base_pos_ptr = new PostureConstraint(
              prob_->getDrakeModel());
          if (cnt == 0)
            qs = q0;
          else
          {
            for (int i = 0; i < drm_drake_joints_map_.size(); i++)
              qs(drm_drake_joints_map_[i]) =
                  drm_client_.getResult()->q_out.data[i];
            tmp_rot = KDL::Rotation::Quaternion(
                drm_client_.getResult()->q_out.data[3],
                drm_client_.getResult()->q_out.data[4],
                drm_client_.getResult()->q_out.data[5],
                drm_client_.getResult()->q_out.data[6]);
            tmp_rot.GetRPY(qs(3), qs(4), qs(5));
            Eigen::VectorXd lb, ub;
            lb = qs.segment<3>(0) - Eigen::Vector3d::Ones() * 0.15;
            ub = qs.segment<3>(0) + Eigen::Vector3d::Ones() * 0.15;
            std::vector<int> idx = { 0, 1, 2 };

            base_pos_ptr->setJointLimits(3, idx.data(), lb, ub);
            prob_->constraints_.push_back(base_pos_ptr);
          }
          std::vector<std::string> infeasible_constraint;
          inverseKin(prob_->getDrakeModel(), qs, qs, prob_->constraints_.size(),
              &prob_->constraints_[0], q_sol, info, infeasible_constraint,
              *ik_options_);
          if (cnt != 0)
          {
            prob_->constraints_.pop_back();
            delete base_pos_ptr;
          }

          if (info != 0 && info != 1 && info != 3 && info != 4 && info != 5 && info != 6 && info != 13)
          {
            for (int i = 0; i < infeasible_constraint.size(); i++)
              ERROR("Infeasible: "<<infeasible_constraint[i]);
            HIGHLIGHT_NAMED(object_name_,
                "Drake IK info="<<info<<", continue to solve");
            cnt++;
            goal.invalid_clusters.push_back(
                drm_client_.getResult()->cluster_index);
            goal.invalid_cluster_cells.push_back(
                drm_client_.getResult()->sample_eff_index);
            continue;
          }
          for (int i = 6; i < prob_->getDrakeModel()->num_positions; i++)
          {
            prob_->getScenes().begin()->second->getPlanningScene()->getCurrentStateNonConst().setVariablePosition(
                prob_->getDrakeModel()->getPositionName(i), q_sol(i));
          }
          KDL::Rotation tmp_rot = KDL::Rotation::RPY(q_sol(3), q_sol(4),
              q_sol(5));
          geometry_msgs::Pose tmp_pose;
          tmp_pose.position.x = q_sol(0);
          tmp_pose.position.y = q_sol(1);
          tmp_pose.position.z = q_sol(2);
          tmp_rot.GetQuaternion(tmp_pose.orientation.x, tmp_pose.orientation.y,
              tmp_pose.orientation.z, tmp_pose.orientation.w);
          Eigen::Affine3d tmp_affine;
          tf::poseMsgToEigen(tmp_pose, tmp_affine);
          prob_->getScenes().begin()->second->getPlanningScene()->getCurrentStateNonConst().setJointPositions(
              "world_joint", tmp_affine);
          prob_->getScenes().begin()->second->getPlanningScene()->getCurrentStateNonConst().update(
              true);
          prob_->getScenes().begin()->second->publishScene();
          collision_detection::CollisionRequest req;
          req.distance = true;
          req.contacts = true;
          req.max_contacts = 100;
          req.group_name = "BaseUpperBodyLeft";
          collision_detection::CollisionResult res;
          prob_->getScenes().begin()->second->getPlanningScene()->checkCollision(
              req, res, prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState());
          succeeded = true;
          if (res.collision)
          {
            bool col = false;
            for (auto&it : res.contacts)
            {
              if (col) break;
              for (auto&it2 : obs_map)
              {
                if ((it.first.first.compare(it2.first) == 0
                    || it.first.second.compare(it2.first) == 0)
                    && (it.first.second.compare("leftPalm") != 0))
                {
                  goal.invalid_clusters.push_back(
                      drm_client_.getResult()->cluster_index);
                  goal.invalid_cluster_cells.push_back(
                      drm_client_.getResult()->sample_eff_index);
                  col = true;
                  succeeded = false;
                  HIGHLIGHT(
                      "Collision between "<<it.first.first<<" and "<<it.first.second<<" size "<<it.second.size()<<" at "<<it.second[0].pos.transpose());
                  geometry_msgs::PoseStamped tmp_pose;
                  tmp_pose.header.frame_id = "world_frame";
                  tmp_pose.header.stamp = ros::Time::now();
                  tmp_pose.pose.position.x = it.second[0].pos(0);
                  tmp_pose.pose.position.y = it.second[0].pos(1);
                  tmp_pose.pose.position.z = it.second[0].pos(2);
                  tmp_pub.publish(tmp_pose);
                  ros::spinOnce();
//                  getchar();
                  break;
                }
              }
            }
          }
        }
        else
        {
          break;
        }
      }
      else
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      if (succeeded) break;
      cnt++;
    }
    if (succeeded)
    {
      solution.resize(1, prob_->getDrakeModel()->num_positions);
      solution.row(0) = q_sol;
      planning_time_ = ros::Duration(ros::Time::now() - start);
      HIGHLIGHT("Succeeded at iteration "<<cnt);
      return SUCCESS;
    }
    else
    {
      solution.resize(1, prob_->getDrakeModel()->num_positions);
      solution.row(0) = q_sol;
      planning_time_ = ros::Duration(ros::Time::now() - start);
      HIGHLIGHT("Failed");
    }

    return SUCCESS;
  }
}
