/*
 * DRM.cpp
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRM.h"
#include <visualization_msgs/MarkerArray.h>

namespace dynamic_reachability_map
{
  DRM::DRM()
      : nh_("~")
  {
    astar_pub_ = nh_.advertise<visualization_msgs::Marker>("AStar", 1);
    astar_mark_.type = visualization_msgs::Marker::CUBE_LIST;
    astar_mark_.header.stamp = ros::Time::now();
    astar_mark_.header.frame_id = "world_frame";
    astar_mark_.scale.x = astar_mark_.scale.y = astar_mark_.scale.z = 0.1;
    astar_mark_.action = visualization_msgs::Marker::ADD;
  }

  DRM::~DRM()
  {

  }

  bool DRM::initialise(const std::string &drms_path,
      const robot_model::RobotModelConstPtr &model)
  {
    space_.reset(new DRMSpace());
    DRMSpaceLoader* drmsl;
    drmsl = new DRMSpaceLoader();
    if (!drmsl->loadSpace(drms_path, space_, model))
    {
      ROS_ERROR_STREAM("Can not load DRM space from path "<<drms_path);
      return false;
    }
    else
    {
      ROS_INFO("DRM space loaded successfully!");
    }

    W_ = Eigen::MatrixXd::Ones(4 + space_->getDimension(),
        4 + space_->getDimension());
    for (int i = 0; i < 4; i++)
      W_(i, i) = 1e2;
    return true;
  }

  const DRMSpace_ptr & DRM::space() const
  {
    return space_;
  }

  DRMSpace_ptr & DRM::spaceNonConst()
  {
    return space_;
  }

  void DRM::resetDRM2FreeSpace()
  {
    boost::mutex::scoped_lock(space_->space_lock_);
    for (unsigned int i = 0; i < space_->space_size_; i++)
      space_->volumes_[i].isFree = true;
    for (unsigned int i = 0; i < space_->sample_size_; i++)
      space_->samples_[i].isValid = true;
    for (unsigned int i = 0; i < space_->edges_.size(); i++)
      space_->edges_[i].isValid = true;
  }

  void DRM::updateOccupation(const std::map<unsigned int, bool> &occup_list)
  {
    if (invalid_volumes_.size() == occup_list.size()
        && std::equal(invalid_volumes_.begin(), invalid_volumes_.end(),
            occup_list.begin())) return;
    for (auto&it : invalid_volumes_)
    {
      space_->volumes_[it.first].isFree = true;
      for (unsigned long int j = 0;
          j < space_->volumes_[it.first].occup_samples.size(); j++)
        space_->samples_[space_->volumes_[it.first].occup_samples[j]].isValid =
            true;
    }
    invalid_volumes_ = occup_list;
    for (auto&it : invalid_volumes_)
    {
      space_->volumes_[it.first].isFree = false;
      for (unsigned long int j = 0;
          j < space_->volumes_[it.first].occup_samples.size(); j++)
        space_->samples_[space_->volumes_[it.first].occup_samples[j]].isValid =
            false;
    }
  }

  dynamic_reachability_map::DRMResult DRM::getIKSolution(
      const dynamic_reachability_map::DRMGoalConstPtr &goal)
  {
    ros::Time start_time = ros::Time::now();
    dynamic_reachability_map::DRMResult result;
    result.succeed = false;
    unsigned int index = 0;
    if (!space_->getVolumeIndex(goal->goal_pose.position, index))
    {
      ROS_WARN_STREAM(
          "Pose ("<<goal->goal_pose.position.x<<","<<goal->goal_pose.position.y<<","<<goal->goal_pose.position.z<<") is not in the reachable space");
      return result;
    }
    unsigned long int sample_index = 0;
    std::vector<unsigned long int> candidate_samples;
    std::vector<std::pair<unsigned int, unsigned int>> candidate_info;
    if (space_->CurrentlyReachability(index, goal->invalid_clusters,
        goal->invalid_cluster_cells, candidate_samples, candidate_info) > 0)
    {
      if (getClosestSample(goal, candidate_samples, sample_index))
        result.succeed = true;
    }
    else
    {
      std::vector<std::pair<unsigned int, double> > neighbours =
          space_->getNeighborIndices(index, 1);
      ROS_WARN_STREAM(
          "Pose ("<<goal->goal_pose.position.x<<","<<goal->goal_pose.position.y<<","<<goal->goal_pose.position.z<<") is not in current environment. Try "<<neighbours.size()<<" neighbours");
      for (unsigned int i = 0; i < neighbours.size(); i++)
      {
        if (space_->CurrentlyReachability(neighbours[i].first,
            goal->invalid_clusters, goal->invalid_cluster_cells,
            candidate_samples, candidate_info) > 0
            && getClosestSample(goal, candidate_samples, sample_index))
        {
          result.succeed = true;
          break;
        }
      }
    }

    if (result.succeed)
    {
      result.sample_index = candidate_samples[sample_index];
      result.sample_eff_index = candidate_info[sample_index].first;
      result.cluster_index = candidate_info[sample_index].second;
      result.q_out.data.resize(space_->getDimension());
      for (int i = 0; i < space_->getDimension(); i++)
        result.q_out.data[i] = space_->getSample(
            candidate_samples[sample_index]).q[i];
    }
    ROS_WARN_STREAM(
        "IK request time: "<<ros::Duration(ros::Time::now()-start_time).toSec()<<"sec");
    return result;
  }

  dynamic_reachability_map::DRMTrajResult DRM::getTrajectory(
      const dynamic_reachability_map::DRMTrajGoalConstPtr &goal)
  {

    dynamic_reachability_map::DRMTrajResult result;
    unsigned int start_space_index = 0, goal_space_index = 0;
    std::vector<unsigned long int> candidate_samples;
    unsigned int index = 0;

    if (!getSampleReachIndex(goal->q0, start_space_index)
        || !getSampleReachIndex(goal->qT, goal_space_index))
    {
      result.succeed = false;
      return result;
    }
    ROS_WARN_STREAM(
        "start index "<<start_space_index<<"("<<space_->at(start_space_index).center.x<<","<<space_->at(start_space_index).center.y<<","<<space_->at(start_space_index).center.z<<"), goal index "<<goal_space_index<<"("<<space_->at(goal_space_index).center.x<<","<<space_->at(goal_space_index).center.y<<","<<space_->at(goal_space_index).center.z<<")");
    std::vector<unsigned int> path;
    if (searchAStar(start_space_index, goal_space_index, path))
    {
      std::cout << "A star path ";
      for (int i = 0; i < path.size(); i++)
        std::cout << path[i] << " ";
      std::cout << std::endl;
      result.solution.resize(path.size() + 2);
      result.solution[0] = goal->q0;
      for (int i = 0; i < path.size(); i++)
      {
        double dist = INFINITY;
        unsigned long int tmp_index = 0;
        Eigen::VectorXd tmp(space_->getDimension());
        for (int k = 0; k < space_->getDimension(); k++)
          tmp(k) = result.solution[i].data[k];
        for (int k = 0; k < 3; k++)
          tmp(k) *= 1e5;
        for (int j = 0; j < space_->at(path[i]).reach_samples.size(); j++)
        {
          if (space_->getSample(space_->at(path[i]).reach_samples[j]).isValid)
          {
            Eigen::VectorXd tmp2(space_->getDimension());
            for (int k = 0; k < space_->getDimension(); k++)
              tmp2(k) =
                  space_->getSample(space_->at(path[i]).reach_samples[j]).q[k];
            for (int k = 0; k < 3; k++)
              tmp2(k) *= 1e5;
            double tmp_dist = (tmp - tmp2).norm();
            if (tmp_dist < dist)
            {
              dist = tmp_dist;
              tmp_index = space_->at(path[i]).reach_samples[j];
            }
          }
        }
        result.solution[i + 1].data.resize(space_->getDimension());
        for (int k = 0; k < space_->getDimension(); k++)
          result.solution[i + 1].data[k] = space_->getSample(tmp_index).q[k];
        result.solution[i + 1].data[2] += 1.025;
      }
      result.solution[path.size() + 1] = goal->qT;
      result.succeed = true;
    }
    else
    {
      result.solution.resize(2);
      result.solution[0] = goal->q0;
      result.solution[1] = goal->qT;
      result.succeed = false;
    }
    return result;
  }

  bool DRM::searchAStar(unsigned int start_space_index,
      unsigned int goal_space_index, std::vector<unsigned int> &space_path)
  {
    ROS_INFO_STREAM("AStar from "<<start_space_index<<" to "<<goal_space_index);
    std::map<unsigned int, double> closed;
    std::map<unsigned int, double> open;
    std::vector<AStarNode> aStar;
    aStar.resize(space_->getSpaceSize());
    aStar[start_space_index].g = 0;
    aStar[start_space_index].h = heuristicCost(start_space_index,
        goal_space_index);
    open[start_space_index] = aStar[start_space_index].f();

    while (open.size() > 0)
    {
      double min = INFINITY;
      std::map<unsigned int, double>::iterator current;
      for (std::map<unsigned int, double>::iterator it = open.begin();
          it != open.end(); it++)
      {
        if (it->second < min)
        {
          current = it;
          min = it->second;
        }
      }
      if (current->first == goal_space_index)
      {
        ROS_INFO("A star succeeded");
        space_path.resize(0);
        int index = current->first;
        space_path.push_back(index);
        while (aStar[index].parent != -1)
        {
          index = aStar[index].parent;
          space_path.push_back(index);
        }
        std::reverse(space_path.begin(), space_path.end());
        ROS_INFO("A star path constructed");
        return true;
      }
//      astar_mark_.colors.clear();
//      astar_mark_.points.clear();
//      std_msgs::ColorRGBA c;
//      c.a = 1;
//      c.g = 1;
//      astar_mark_.colors.push_back(c);
//      geometry_msgs::Point tmp = space_->at(current->first).center;
//      tmp.z += 1.025;
//      astar_mark_.points.push_back(tmp);

      unsigned int tmp_index = current->first;
      closed[current->first] = current->second;
      open.erase(current);
      current = closed.find(tmp_index);
      std::vector<std::pair<unsigned int, double> > neighbors =
          space_->getNeighborIndices(current->first, 1);
      if (neighbors.size() == 0)
      {
        ROS_ERROR_STREAM(
            "Volume "<<current->first<<"("<<space_->at(current->first).center.x<<","<<space_->at(current->first).center.y<<","<<space_->at(current->first).center.z<<") has no neighbors");
        continue;
      }
//      else
//      {
//        for (int i = 0; i < neighbors.size(); i++)
//        {
//          std_msgs::ColorRGBA cc;
//          cc.a = 0.5;
//          cc.r = 1;
//          astar_mark_.colors.push_back(cc);
//          geometry_msgs::Point tmp2 = space_->at(neighbors[i].first).center;
//          tmp2.z += 1.025;
//          astar_mark_.points.push_back(tmp2);
//        }
//      }
//      astar_pub_.publish(astar_mark_);
//      ros::spinOnce();
//      ros::Duration(1).sleep();
      for (int i = 0; i < neighbors.size(); i++)
      {
        if (closed.find(neighbors[i].first) == closed.end())
        {
          double tentative_g = aStar[current->first].g + neighbors[i].second;
          if (open.find(neighbors[i].first) == open.end()
              || tentative_g < aStar[neighbors[i].first].g)
          {
            aStar[neighbors[i].first].parent = current->first;
            aStar[neighbors[i].first].g = tentative_g;
            double h_dist = heuristicCost(neighbors[i].first, goal_space_index);
            aStar[neighbors[i].first].h = h_dist;
            if (open.find(neighbors[i].first) == open.end())
              open[neighbors[i].first] = tentative_g + h_dist;
          }
        }
      }
    }
    ROS_WARN("AStart reaches end, failed");
    return false;
  }

  double DRM::heuristicCost(unsigned int a, unsigned int b)
  {
    Eigen::Vector3d ea, eb;
    ea << space_->at(a).center.x, space_->at(a).center.y, space_->at(a).center.z;
    eb << space_->at(b).center.x, space_->at(b).center.y, space_->at(b).center.z;
    return 1e1 * sqrt((ea - eb).cwiseAbs2().sum());
  }
  bool DRM::getClosestSample(
      const dynamic_reachability_map::DRMGoalConstPtr &goal,
      std::vector<unsigned long int> &candidate_samples,
      unsigned long int &sample_index, bool use_invalids)
  {
    if (candidate_samples.size() == 0) return false;
    double dist = INFINITY;
    Eigen::VectorXf e1(4 + space_->getDimension());
    e1(0) = goal->goal_pose.orientation.x;
    e1(1) = goal->goal_pose.orientation.y;
    e1(2) = goal->goal_pose.orientation.z;
    e1(3) = goal->goal_pose.orientation.w;
    for (int i = 0; i < space_->getDimension(); i++)
      e1(i + 4) = 0 * goal->q0.data[i];
    for (unsigned long int i = 0; i < candidate_samples.size(); i++)
    {

      Eigen::VectorXf e2(4 + space_->getDimension());
      static double w = 1e3;
      e2(0) = w * space_->getSample(candidate_samples[i]).effpose.orientation.x;
      e2(1) = w * space_->getSample(candidate_samples[i]).effpose.orientation.y;
      e2(2) = w * space_->getSample(candidate_samples[i]).effpose.orientation.z;
      e2(3) = w * space_->getSample(candidate_samples[i]).effpose.orientation.w;
      for (int j = 0; j < space_->getDimension(); j++)
        e2(4 + j) = 0 * space_->getSample(candidate_samples[i]).q[j];
      double tmp_dist = (e1 - e2).norm();
      if (tmp_dist < dist)
      {
        dist = tmp_dist;
        sample_index = i;
      }
    }
    return true;
  }

  bool DRM::getClosestSample(const exotica::Vector &q,
      unsigned long int &sample_index, unsigned int &space_index)
  {
    Eigen::VectorXf q_eigen(q.data.size());
    for (int i = 0; i < space_->dimension_; i++)
    {
      space_->ps_->getCurrentStateNonConst().setVariablePosition(
          space_->var_index_[i], q.data[i]);
      q_eigen(i) = q.data[i];
    }
    space_->ps_->getCurrentStateNonConst().update(true);

    Eigen::Affine3d effpose =
        space_->ps_->getCurrentStateNonConst().getGlobalLinkTransform(
            space_->eff_);
    if (!space_->getVolumeIndex(effpose, space_index))
    {
      ROS_ERROR("Get volume index failed");
      return false;
    }
    double dist = INFINITY;
    for (unsigned long int i = 0;
        i < space_->volumes_[space_index].reach_samples.size(); i++)
    {
      double tmp_dist = (Eigen::Map<Eigen::VectorXf>(
          space_->samples_[space_->volumes_[space_index].reach_samples[i]].q,
          space_->dimension_) - q_eigen).norm();
      if (tmp_dist < dist)
      {
        dist = tmp_dist;
        sample_index = space_->volumes_[space_index].reach_samples[i];
      }
    }
    return true;
  }

  bool DRM::getSampleReachIndex(const exotica::Vector &q,
      unsigned int &space_index)
  {
    Eigen::VectorXf q_eigen(q.data.size());
    for (int i = 0; i < space_->dimension_; i++)
    {
      space_->ps_->getCurrentStateNonConst().setVariablePosition(
          space_->var_index_[i], q.data[i]);
      q_eigen(i) = q.data[i];
    }
    space_->ps_->getCurrentStateNonConst().update(true);
    Eigen::Affine3d effpose =
        space_->ps_->getCurrentStateNonConst().getGlobalLinkTransform(
            space_->eff_);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(effpose, pose);
    pose.position.z -= 1.025;
    if (!space_->getVolumeIndex(pose.position, space_index))
    {
      ROS_ERROR_STREAM(
          "Get volume index failed Index "<<space_index<<" ("<<effpose.translation().transpose()<<")");
      return false;
    }
    return true;
  }
}

