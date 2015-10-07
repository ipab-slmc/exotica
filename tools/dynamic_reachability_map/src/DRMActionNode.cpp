/*
 * DRMActionNode.cpp
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMActionNode.h"
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "dynamic_reachability_map/DRMSpaceSaver.h"

namespace dynamic_reachability_map
{
  DRMActionNode::DRMActionNode()
      : nh_("~"), as_(nh_, "/DRM_IK",
          boost::bind(&dynamic_reachability_map::DRMActionNode::getIKSolution,
              this, _1), false)
  {
  }

  DRMActionNode::~DRMActionNode()
  {
    if (drm_cluster_) delete drm_cluster_;
  }

  bool DRMActionNode::initialise()
  {
    std::string model, drm_model, path;
    if (!nh_.hasParam("RobotModel"))
      model = "robot_description";
    else
      nh_.getParam("RobotModel", model);
    if (!nh_.getParam("DRMModel", drm_model))
    {
      ROS_ERROR(
          "Dynamic reachability map urdf model not defined [rosparam: DRMModel]");
      return false;
    }
    if (!nh_.getParam("DRMPath", path))
    {
      ROS_ERROR(
          "Dynamic reachability map files not defined [rosparam: DRMPath]");
      return false;
    }

    robot_model_loader::RobotModelLoader robot_model_loader(model);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    drm_.reset(new DRM());
    if (!drm_->initialise(path, kinematic_model))
    {
      return false;
    }

    bool clustering = false;
    if (nh_.hasParam("Clustering")) nh_.getParam("Clustering", clustering);
    if (clustering)
    {
      drm_cluster_ = new DRMSampleCluster();
      DRMClusterParam cluster_param;
      drm_cluster_->startClustering(drm_->spaceNonConst(), cluster_param);
      delete drm_cluster_;
//    std::string path = ros::package::getPath("dynamic_reachability_map") + "/result2";
//    std::string savepath;
//    ROS_INFO_STREAM("Saving sampling result to "<<path);
//    dynamic_reachability_map::DRMSpaceSaver saver;
//    saver.saveSpace(path, drm_->spaceNonConst(), savepath);
    }
//  drm_->space()->buildConnectionGraph( M_PI / 4);
//  drm_timer_ = nh_.createTimer(ros::Duration(1), &DRMActionNode::drmTimeCallback, this);
//  if (clustering)
//  drm_state_timer_ = nh_.createTimer(ros::Duration(1), &DRMActionNode::drmStateTimeCallback, this);

    traj_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("ClusterCenters",
        1);
    disp_traj_.model_id =
        drm_->space()->getPlanningScene()->getRobotModel()->getName();
    disp_traj_.trajectory.resize(1);
    disp_traj_.trajectory[0].joint_trajectory.header.frame_id = "world_frame";
    disp_traj_.trajectory[0].joint_trajectory.joint_names =
        drm_->space()->getGroup()->getVariableNames();

    astar_traj_ = disp_traj_;
    astar_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(
        "AStarTrajectory", 1);

    robot_model_loader::RobotModelLoader cellbot_loader(drm_model);
    robot_model::RobotModelPtr cellbot_model = cellbot_loader.getModel();
    cell_ps_.reset(new planning_scene::PlanningScene(cellbot_model));
    cell_ps_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(
        "CellPlanningScene", 10);
    state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("DRMState", 10);
    drm_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "DynamicReachabilityMap", 10);

    drm_mark_.type = visualization_msgs::Marker::CUBE_LIST;
    drm_mark_.header.stamp = ros::Time::now();
    drm_mark_.header.frame_id = "world_frame";
    drm_mark_.scale.x = drm_mark_.scale.y = drm_mark_.scale.z =
        drm_->space()->getResolution();
    drm_mark_.action = visualization_msgs::Marker::ADD;

    graph_markers_.markers.resize(2);
    graph_markers_.markers[0].type = visualization_msgs::Marker::POINTS;
    graph_markers_.markers[0].id = 0;
    graph_markers_.markers[0].header.stamp = ros::Time::now();
    graph_markers_.markers[0].header.frame_id = "world_frame";
    graph_markers_.markers[0].scale.x = graph_markers_.markers[0].scale.y =
        0.002;
    graph_markers_.markers[0].color.a = graph_markers_.markers[0].color.r = 1;
    graph_markers_.markers[0].action = visualization_msgs::Marker::ADD;

    graph_markers_.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    graph_markers_.markers[1].id = 1;
    graph_markers_.markers[1].header.stamp = ros::Time::now();
    graph_markers_.markers[1].header.frame_id = "world_frame";
    graph_markers_.markers[1].scale.x = 0.0005;
    graph_markers_.markers[1].color.a = .6;
    graph_markers_.markers[1].color.b = .8;
    graph_markers_.markers[1].action = visualization_msgs::Marker::ADD;
    graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "ConnectivityGraph", 1);

    graph_markers_.markers[0].points.resize(drm_->space()->getSampleSize());
    graph_markers_.markers[0].colors.resize(drm_->space()->getSampleSize());
    std_msgs::ColorRGBA c0, c1, c2;
    c0.a = c0.g = 1;
    c1.a = c1.r = 1;
    c2.a = 0.6;
    c2.b = 0.8;
    for (unsigned long int i = 0; i < drm_->space()->getSampleSize(); i++)
    {
      graph_markers_.markers[0].points[i] =
          drm_->space()->getSample(i).effpose.position;
      if (drm_->space()->getSample(i).edges.size() > 0)
        graph_markers_.markers[0].colors[i] = c0;
      else
        graph_markers_.markers[0].colors[i] = c1;
    }

    graph_markers_.markers[1].points.resize(2 * drm_->space()->edges_.size());
    graph_markers_.markers[1].colors.resize(2 * drm_->space()->edges_.size());

    for (unsigned long int i = 0; i < drm_->space()->edges_.size(); i++)
    {
      graph_markers_.markers[1].points[2 * i] = drm_->space()->getSample(
          drm_->space()->edges_[i].a).effpose.position;
      graph_markers_.markers[1].points[2 * i + 1] = drm_->space()->getSample(
          drm_->space()->edges_[i].b).effpose.position;
      graph_markers_.markers[1].colors[2 * i] = c2;
      graph_markers_.markers[1].colors[2 * i + 1] = c2;

    }
//  graph_timer_ = nh_.createTimer(ros::Duration(2), &DRMActionNode::graphTimeCallback, this);
    as_.start();

    return true;
  }

  bool DRMActionNode::updateDRM(
      const dynamic_reachability_map::DRMGoalConstPtr &goal,
      const KDL::Frame &base_pose)
  {
    drm_->resetDRM2FreeSpace();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1;
    collision_detection::CollisionResult result;

    moveit_msgs::PlanningScene msg;
    cell_ps_->getPlanningSceneMsg(msg);
    cell_ps_pub_.publish(msg);
    for (unsigned int i = 0; i < drm_->space()->getSpaceSize(); i++)
    {
      KDL::Frame cell_pose = point2KDL(drm_->space()->at(i).center);
      cell_pose = base_pose * cell_pose;
      cell_ps_->getCurrentStateNonConst().setVariablePosition(0,
          cell_pose.p.x());
      cell_ps_->getCurrentStateNonConst().setVariablePosition(1,
          cell_pose.p.y());
      cell_ps_->getCurrentStateNonConst().setVariablePosition(2,
          cell_pose.p.z());
      std::vector<double> quat(4);
      cell_pose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
      cell_ps_->getCurrentStateNonConst().setVariablePosition(3, quat[0]);
      cell_ps_->getCurrentStateNonConst().setVariablePosition(4, quat[1]);
      cell_ps_->getCurrentStateNonConst().setVariablePosition(5, quat[2]);
      cell_ps_->getCurrentStateNonConst().setVariablePosition(6, quat[3]);
      cell_ps_->getCurrentStateNonConst().update(true);
      cell_ps_->checkCollision(request, result);
      if (result.collision)
      {
        drm_->space()->setVolumeOccupied(i, true);
      }
      result.clear();

    }

//  std::vector<unsigned int> density = drm_->space()->getVolumeReachabilities();
//
//  int size = density.size();
//  Eigen::VectorXd density_eigen(size);
//  for (unsigned int i = 0; i < size; i++)
//    density_eigen(i) = density[i];
//  double max = density_eigen.maxCoeff();
//  density_eigen = density_eigen / max;
//
//  drm_mark_.colors.clear();
//  drm_mark_.points.clear();
//  unsigned int goal_index = 0;
//  if (drm_->space()->getVolumeIndex(goal->goal_pose.position, goal_index))
//  {
//    std_msgs::ColorRGBA c;
//    c.a = c.r = 1;
//    drm_mark_.colors.push_back(c);
//    drm_mark_.points.push_back(drm_->space()->at(goal_index).center);
//  }
//
//  for (unsigned int i = 0; i < size; i++)
//  {
//    if (density[i] > 0)
//    {
//      std_msgs::ColorRGBA c;
//      c.a = density_eigen(i) + 0.1;
//      c.g = density_eigen(i);
//      drm_mark_.colors.push_back(c);
//      drm_mark_.points.push_back(drm_->space()->at(i).center);
//    }
//  }
//  if (drm_mark_.points.size() == 0)
//    ROS_ERROR("Empty DRM!!!!");
//  drm_pub_.publish(drm_mark_);
    return true;
  }
  bool DRMActionNode::getIKSolution(
      const dynamic_reachability_map::DRMGoalConstPtr &goal)
  {
    static bool first = true;
    if (first)
    {
      disp_state_.state = goal->ps.robot_state;
      first = false;
    }
    ros::Time update_start = ros::Time::now();
    moveit_msgs::PlanningSceneWorld world = goal->ps.world;
    for (int i = 0; i < world.collision_objects.size(); i++)
      world.collision_objects[i].header.frame_id = "/world_frame";
    cell_ps_->getWorldNonConst()->clearObjects();
    cell_ps_->processPlanningSceneWorldMsg(world);
    ROS_INFO_STREAM(
        "DRM ps time "<<ros::Duration(ros::Time::now()-update_start).toSec()<<"sec");
    update_start = ros::Time::now();
    updateDRM(goal);
    ROS_INFO_STREAM(
        "DRM update time "<<ros::Duration(ros::Time::now()-update_start).toSec()<<"sec");
    dynamic_reachability_map::DRMResult result = drm_->getIKSolution(goal);
    as_.setSucceeded(result);
    for (int i = 0; i < result.q_out.data.size(); i++)
      disp_state_.state.joint_state.position[i] = result.q_out.data[i];
    Eigen::VectorXd q0(drm_->space()->getDimension());
    for (int i = 0; i < drm_->space()->getDimension(); i++)
      q0(i) = goal->q0.data[i];
    state_pub_.publish(disp_state_);
    graphTimeCallback(ros::TimerEvent());
    if (false && result.succeed)
    {
      ros::Time astar_start = ros::Time::now();
      std::vector<unsigned long int> path;
      if (drm_->solve(goal, result.sample_index, path))
      {
        ROS_INFO_STREAM(
            "A* succeeded in "<<ros::Duration(ros::Time::now()-astar_start).toSec()<<" path length "<<path.size());
        for (int i = 0; i < path.size(); i++)
        {
          std::cout << "Path " << i << ": " << path[i] << " ";
          for (int d = 0; d < 7; d++)
            std::cout << drm_->space()->getSample(path[i]).q[d] << " ";
          std::cout << std::endl;
        }
        astar_traj_.trajectory[0].joint_trajectory.header.stamp =
            ros::Time::now();
        astar_traj_.trajectory[0].joint_trajectory.points.resize(path.size());
        for (int i = 0; i < path.size(); i++)
        {
          astar_traj_.trajectory[0].joint_trajectory.points[i].positions.resize(
              drm_->space()->getDimension());
          for (int j = 0; j < drm_->space()->getDimension(); j++)
          {
            astar_traj_.trajectory[0].joint_trajectory.points[i].positions[j] =
                drm_->space()->getSample(path[i]).q[j];
          }
        }
        astar_traj_.trajectory_start.joint_state.name =
            astar_traj_.trajectory[0].joint_trajectory.joint_names;
        astar_traj_.trajectory_start.joint_state.position =
            astar_traj_.trajectory[0].joint_trajectory.points[path.size() - 1].positions;

        astar_pub_.publish(astar_traj_);
      }
      else
      {
        ROS_ERROR("A* failed");
      }
    }
    return result.succeed;
  }

  void DRMActionNode::drmTimeCallback(const ros::TimerEvent& event)
  {
    static int cnt = 0;
    std::vector<unsigned int> density =
        drm_->space()->getVolumeReachabilities();
    int size = density.size();
    Eigen::VectorXd density_eigen(size);
    for (int i = 0; i < size; i++)
      density_eigen(i) = density[i];
    double max = density_eigen.maxCoeff();
    density_eigen = density_eigen / max;

    drm_mark_.colors.clear();
    drm_mark_.points.clear();
    for (unsigned int i = 0; i < size; i++)
    {
      if (density[i] > 0)
      {
        std_msgs::ColorRGBA c;
        c.a = density_eigen(i);
        c.r = 1 - density_eigen(i);
        c.g = 1;
        c.b = density_eigen(i);
        drm_mark_.colors.push_back(c);
        drm_mark_.points.push_back(drm_->space()->at(i).center);
      }
    }
    if (drm_mark_.points.size() == 0)
    ROS_ERROR("Empty DRM!!!!");
    drm_pub_.publish(drm_mark_);
  }

  void DRMActionNode::drmStateTimeCallback(const ros::TimerEvent& event)
  {
    static unsigned int space_index = 0;
    if (space_index == drm_->space()->getSpaceSize()) space_index = 0;
    while (drm_->space()->at(space_index).reach_samples.size() <= 0)
    {
      space_index++;
      if (space_index == drm_->space()->getSpaceSize()) space_index = 0;
    }
    ROS_INFO_STREAM(
        "Publishing Volume "<<space_index<<": "<<drm_->space()->at(space_index).reach_samples.size()<<" clusters");
    disp_traj_.trajectory[0].joint_trajectory.header.stamp = ros::Time::now();
    disp_traj_.trajectory[0].joint_trajectory.points.resize(
        drm_->space()->at(space_index).reach_samples.size());
    for (int i = 0; i < drm_->space()->at(space_index).reach_samples.size();
        i++)
    {
      disp_traj_.trajectory[0].joint_trajectory.points[i].positions.resize(
          drm_->space()->getDimension());
      for (int j = 0; j < drm_->space()->getDimension(); j++)
      {
        disp_traj_.trajectory[0].joint_trajectory.points[i].positions[j] =
            drm_->space()->getSample(
                drm_->space()->at(space_index).reach_samples[i]).q[j];
      }
    }
    disp_traj_.trajectory_start.joint_state.name =
        disp_traj_.trajectory[0].joint_trajectory.joint_names;
    disp_traj_.trajectory_start.joint_state.position =
        disp_traj_.trajectory[0].joint_trajectory.points[drm_->space()->at(
            space_index).reach_samples.size() - 1].positions;

    traj_pub_.publish(disp_traj_);
    space_index++;
  }

  void DRMActionNode::graphTimeCallback(const ros::TimerEvent& event)
  {
    for (unsigned long int i = 0; i < drm_->space()->getSampleSize(); i++)
    {
      graph_markers_.markers[0].colors[i].a =
          drm_->space()->getSample(i).isValid ? 1 : 0;

    }

    for (unsigned long int i = 0; i < drm_->space()->edges_.size(); i++)
    {
      graph_markers_.markers[1].colors[2 * i].a =
          graph_markers_.markers[1].colors[2 * i + 1].a =
              drm_->space()->edges_[i].isValid ? 0.6 : 0;
    }
    graph_pub_.publish(graph_markers_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "~");
  ros::AsyncSpinner sp(1);
  dynamic_reachability_map::DRMActionNode drm;
  if (!drm.initialise())
  {
    ROS_ERROR("ERROR initialise dynamic reachability map");
    return 0;
  }
  sp.start();
  ros::waitForShutdown();
  return 0;
}
