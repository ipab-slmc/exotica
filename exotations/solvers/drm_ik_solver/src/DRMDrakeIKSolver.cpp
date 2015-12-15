/*
 * DRMDrakeIK.cpp
 *
 *  Created on: 13 Oct 2015
 *      Author: yiming
 */

#include "drm_ik_solver/DRMDrakeIKSolver.h"

REGISTER_MOTIONSOLVER_TYPE("DRMDrakeIKsolver", exotica::DRMDrakeIKsolver);
namespace exotica
{
  DRMDrakeIKsolver::DRMDrakeIKsolver()
      : drm_client_("/DRM_IK", true)
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
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("UseDRM");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, use_drm_);

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
    if (use_drm_->data)
    {
      if (!drm_client_.waitForServer(ros::Duration(2)))
      {
        WARNING_NAMED(object_name_,
            "Can not connect to dynamic reachability map server, manually set to random restart mode");
        use_drm_->data = false;
      }
      else
        HIGHLIGHT_NAMED(object_name_, "DRM server connected.");
    }
    else
      HIGHLIGHT("DRM IK is running is random restart mode")
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
    group_ =
        prob_->getScenes().begin()->second->getPlanningScene()->getRobotModel()->getJointModelGroup(
            "BaseUpperBodyLeft");
    return SUCCESS;
  }
  EReturn DRMDrakeIKsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    if (prob_->getDrakeModel()->num_positions != q0.rows())
    {
      WARNING(
          "Joints mis match Drake model has "<<prob_->getDrakeModel()->num_positions<<" joints, q0 size = "<<q0.rows());
    }
    dynamic_reachability_map::DRMGoal goal;
    Eigen::VectorXd qs = q0, q_sol(q0.rows());
    moveit_msgs::PlanningScene msg;
    prob_->scenes_.begin()->second->getPlanningScene()->getPlanningSceneMsg(
        msg);
    std::map<std::string, bool> obs_map;
    for (int i = 0; i < msg.world.collision_objects.size(); i++)
      obs_map[msg.world.collision_objects[i].id] = true;
    ros::Time start = ros::Time::now();
    int cnt = -1;
    bool succeeded = false;
    while (cnt < 100)
    {
      cnt++;
//      if (cnt == 0)
//        qs = q0;
//      else
      if (!use_drm_->data)
      {
        prob_->getScenes().begin()->second->getPlanningScene()->getCurrentStateNonConst().setToRandomPositions(
            group_);
        KDL::Rotation tmp_rot(
            KDL::Rotation::Quaternion(
                prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                    "world_joint/rot_x"),
                prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                    "world_joint/rot_y"),
                prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                    "world_joint/rot_z"),
                prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                    "world_joint/rot_w")));
        tmp_rot.GetRPY(qs(3), qs(4), qs(5));
        for (int i = 7; i < drm_joints_->strings.size(); i++)
          qs(drm_drake_joints_map_[i]) =
              prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                  drm_joints_->strings[i]);
      }
      else
      {
        goal.ps = msg;
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

        if (drm_client_.sendGoalAndWait(goal, ros::Duration(2)).state_
            == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          if (drm_client_.getResult()->succeed)
          {
            PostureConstraint* base_pos_ptr = new PostureConstraint(
                prob_->getDrakeModel());

            for (int i = 0; i < drm_drake_joints_map_.size(); i++)
              qs(drm_drake_joints_map_[i]) =
                  drm_client_.getResult()->q_out.data[i];
            tmp_rot = KDL::Rotation::Quaternion(
                drm_client_.getResult()->q_out.data[3],
                drm_client_.getResult()->q_out.data[4],
                drm_client_.getResult()->q_out.data[5],
                drm_client_.getResult()->q_out.data[6]);
            tmp_rot.GetRPY(qs(3), qs(4), qs(5));
          }
          else
          {
            WARNING_NAMED(object_name_,
                "DRM failed at iteration "<<cnt<<", switch to random restart mode");
            use_drm_->data = false;
            continue;
          }
        }
        else
        {
          INDICATE_FAILURE
          return FAILURE;
        }
      }
      std::vector<std::string> infeasible_constraint;
      inverseKin(prob_->getDrakeModel(), qs, qs, prob_->constraints_.size(),
          &prob_->constraints_[0], q_sol, info, infeasible_constraint,
          *ik_options_);

      if (info != 0 && info != 1 && info != 3 && info != 4 && info != 5
          && info != 6)
      {
//        for (int i = 0; i < infeasible_constraint.size(); i++)
//          ERROR("Infeasible: "<<infeasible_constraint[i]);
//        HIGHLIGHT_NAMED(object_name_,
//            "Drake IK info="<<info<<", continue to solve");
        if (use_drm_->data && cnt > 0)
        {
          goal.invalid_clusters.push_back(
              drm_client_.getResult()->cluster_index);
          goal.invalid_cluster_cells.push_back(
              drm_client_.getResult()->sample_eff_index);
        }
      }
      else
      {
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
        bool outofbounds = false;
        for (int i = 7; i < drm_joints_->strings.size(); i++)
        {
          if (prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
              drm_ps_joints_map_[i])
              < prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getJointModel(
                  drm_joints_->strings[i])->getVariableBounds()[0].min_position_
              || prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getVariablePosition(
                  drm_ps_joints_map_[i])
                  > prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState().getJointModel(
                      drm_joints_->strings[i])->getVariableBounds()[0].max_position_)
          {
            ROS_ERROR_STREAM(drm_joints_->strings[i]<<" is out of bound");
            outofbounds = true;
          }
          if (outofbounds) break;
        }
        if (outofbounds) continue;
        prob_->getScenes().begin()->second->publishScene();
        collision_detection::CollisionRequest req;
        req.distance = true;
        req.contacts = true;
        req.max_contacts = 100;
        req.group_name = "BaseUpperBodyLeft";
        collision_detection::CollisionResult res;
        prob_->getScenes().begin()->second->getPlanningScene()->checkCollision(
            req, res,
            prob_->getScenes().begin()->second->getPlanningScene()->getCurrentState());
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
                if (use_drm_->data && cnt > 0)
                {
                  goal.invalid_clusters.push_back(
                      drm_client_.getResult()->cluster_index);
                  goal.invalid_cluster_cells.push_back(
                      drm_client_.getResult()->sample_eff_index);
                }
                col = true;
                succeeded = false;
                break;
              }
            }
          }
        }

      }
      if (succeeded) break;
    }
    trials_ = cnt;
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
