/*
 * DRM.cpp
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRM.h"

namespace dynamic_reachability_map
{
  DRM::DRM()
  {

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
    if (space_->CurrentlyReachability(index, candidate_samples) > 0)
    {
      if (getClosestSample(goal, candidate_samples, sample_index))
        result.succeed = true;
    }
    else
    {
      ROS_WARN_STREAM(
          "Pose ("<<goal->goal_pose.position.x<<","<<goal->goal_pose.position.y<<","<<goal->goal_pose.position.z<<") is not in current environment. Try neighbours");
      std::vector<std::pair<unsigned int, double> > neighbours =
          space_->getNeighborIndices(index, 1);
      for (unsigned int i = 0; i < neighbours.size(); i++)
      {
        if (space_->CurrentlyReachability(neighbours[i].first,
            candidate_samples) > 0
            && getClosestSample(goal, candidate_samples, sample_index))
        {
          result.succeed = true;
          break;
        }
      }
    }

    if (result.succeed)
    {
      result.sample_index = sample_index;
      result.q_out.data.resize(space_->getDimension());
      for (int i = 0; i < space_->getDimension(); i++)
        result.q_out.data[i] = space_->getSample(sample_index).q[i];
    }
    ROS_WARN_STREAM(
        "IK request time: "<<ros::Duration(ros::Time::now()-start_time).toSec()<<"sec");
    return result;
  }

  bool DRM::solve(const dynamic_reachability_map::DRMGoalConstPtr &goal,
      unsigned long int goal_sample_index, std::vector<unsigned long int> &path)
  {

    unsigned long int start_sample_index = 0;
    std::vector<unsigned long int> candidate_samples;
    unsigned int index = 0;

    exotica::Vector q0;
    q0.data.resize(7);
    for (int i = 0; i < 7; i++)
      q0.data[i] = 0;
    if (!getClosestSample(q0, start_sample_index)) return false;
    ROS_WARN_STREAM(
        "Start Pose "<<space_->getSample(start_sample_index).effpose.position.x<<","<<space_->getSample(start_sample_index).effpose.position.y<<","<<space_->getSample(start_sample_index).effpose.position.z);
    return searchAStar(start_sample_index, goal_sample_index, path);
  }

  bool DRM::searchAStar(unsigned long int start_sample_index,
      unsigned long int goal_sample_index, std::vector<unsigned long int> &path)
  {
    ROS_INFO_STREAM(
        "AStar from "<<start_sample_index<<" to "<<goal_sample_index);
    //https://en.wikipedia.org/wiki/A*_search_algorithm
    bool succeeded = false;
    std::map<unsigned long int, bool> closed;
    std::map<unsigned long int, bool> open;
    std::map<unsigned long int, double> g_score;
    std::map<unsigned long int, double> f_score;
    std::map<unsigned long int, unsigned long int> came_from;
    std::map<unsigned int, bool> closed_volume;

    g_score[start_sample_index] = 0;
    f_score[start_sample_index] = g_score[start_sample_index]
        + heuristicCost(start_sample_index, goal_sample_index);
    open[start_sample_index] = true;
    unsigned long int current_index = start_sample_index;
    while (open.size() > 0)
    {
      std::map<unsigned long int, bool>::iterator current = open.begin();

      for (std::map<unsigned long int, bool>::iterator it = open.begin();
          it != open.end(); it++)
      {
        if (current->second < it->second) current = it;
      }
      current_index = current->first;
      unsigned int current_volume;
      space_->getVolumeIndex(space_->getSample(current_index).effpose.position,
          current_volume);
      if (current_index == goal_sample_index)
      {
        succeeded = true;
        break;
      }
      closed[current->first] = true;
      open.erase(current);

      std::vector<unsigned long int> edges =
          space_->samples_[current_index].edges;
//    ROS_INFO_STREAM("Sample "<<current_index<<" has "<<edges.size()<<" edges");
      for (unsigned int i = 0; i < edges.size(); i++)
      {
//      ROS_INFO_STREAM("Edge "<<edges[i]<<" a="<<space_->edges_[edges[i]].a<<",b="<<space_->edges_[edges[i]].b);
        unsigned long int neighbour_index =
            current_index == space_->edges_[edges[i]].a ?
                space_->edges_[edges[i]].b : space_->edges_[edges[i]].a;
//      ROS_INFO_STREAM("Sample "<<current_index<<" process edge "<<i<<"/"<<edges.size()<<" Neighbour "<<neighbour_index);

        if (closed.find(neighbour_index) == closed.end())
        {
          double tentative_g = g_score[current_index]
              + 0.1 * space_->edges_[edges[i]].length;

          if (open.find(neighbour_index) == open.end()
              || tentative_g < g_score.at(neighbour_index))
          {
//          ROS_INFO_STREAM("Process "<<neighbour_index);
            came_from[neighbour_index] = current_index;
            g_score[neighbour_index] = tentative_g;
            f_score[neighbour_index] = tentative_g
                + heuristicCost(neighbour_index, goal_sample_index);
            if (open.find(neighbour_index) == open.end())
            {

              unsigned int neighbour_volume;
              space_->getVolumeIndex(
                  space_->getSample(neighbour_index).effpose.position,
                  neighbour_volume);
//            ROS_INFO_STREAM("Neighbour "<<neighbour_index<<" added");
              if (closed_volume.find(neighbour_volume) == closed_volume.end())
                open[neighbour_index] = true;
            }
          }
        }
      }

      closed_volume[current_volume] = true;
    }

    path.clear();
    if (succeeded)
    {
      path.push_back(current_index);
      while (came_from.find(current_index) != came_from.end())
      {
        current_index = came_from.at(current_index);
        path.push_back(current_index);
      }
      ROS_INFO_STREAM("Camefrom "<<came_from.size()<<" path "<<path.size());
      std::reverse(path.begin(), path.end());
    }

    return succeeded;
  }

  double DRM::heuristicCost(unsigned long int a, unsigned long int b)
  {
    Eigen::VectorXd ea(7), eb(7);
    ea << space_->samples_[a].effpose.position.x, space_->samples_[a].effpose.position.y, space_->samples_[a].effpose.position.z, space_->samples_[a].effpose.orientation.x, space_->samples_[a].effpose.orientation.y, space_->samples_[a].effpose.orientation.z, space_->samples_[a].effpose.orientation.w;
    eb << space_->samples_[b].effpose.position.x, space_->samples_[b].effpose.position.y, space_->samples_[b].effpose.position.z, space_->samples_[b].effpose.orientation.x, space_->samples_[b].effpose.orientation.y, space_->samples_[b].effpose.orientation.z, space_->samples_[b].effpose.orientation.w;

    return (ea - eb).norm();
//  return (Eigen::Map<Eigen::VectorXd>(space_->samples_[a].q, space_->dimension_)
//      - Eigen::Map<Eigen::VectorXd>(space_->samples_[b].q, space_->dimension_)).norm();
  }
  bool DRM::getClosestSample(
      const dynamic_reachability_map::DRMGoalConstPtr &goal,
      std::vector<unsigned long int> &candidate_samples,
      unsigned long int &sample_index, bool use_invalids)
  {

    double dist = INFINITY;
    bool invalid = false;
    Eigen::VectorXf e1(4 + space_->getDimension());
    e1(0) = goal->goal_pose.orientation.x;
    e1(1) = goal->goal_pose.orientation.y;
    e1(2) = goal->goal_pose.orientation.z;
    e1(3) = goal->goal_pose.orientation.w;
    for (int i = 0; i < space_->getDimension(); i++)
      e1(i + 4) = goal->q0.data[i];
    for (unsigned long int i = 0; i < candidate_samples.size(); i++)
    {
      invalid = false;
      if (use_invalids) for (int j = 0; j < goal->invalid_samples.size(); j++)
        if (goal->invalid_samples[j] == candidate_samples[i])
        {
          invalid = true;
          break;
        }
      if (!invalid)
      {
        Eigen::VectorXf e2(4 + space_->getDimension());
        static double w = 1e3;
        e2(0) = w
            * space_->getSample(candidate_samples[i]).effpose.orientation.x;
        e2(1) = w
            * space_->getSample(candidate_samples[i]).effpose.orientation.y;
        e2(2) = w
            * space_->getSample(candidate_samples[i]).effpose.orientation.z;
        e2(3) = w
            * space_->getSample(candidate_samples[i]).effpose.orientation.w;
        for (int j = 0; j < space_->getDimension(); j++)
          e2(4 + j) = space_->getSample(candidate_samples[i]).q[j];
        double tmp_dist = (e1 - e2).norm();
        if (tmp_dist < dist)
        {
          dist = tmp_dist;
          sample_index = candidate_samples[i];
        }
      }
    }
    return !invalid;
  }

  bool DRM::getClosestSample(const exotica::Vector &q,
      unsigned long int &sample_index)
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
    unsigned int volume_index = 0;
    if (!space_->getVolumeIndex(effpose, volume_index))
    {
      ROS_ERROR("Get volume index failed");
      return false;
    }
    double dist = INFINITY;
    for (unsigned long int i = 0;
        i < space_->volumes_[volume_index].reach_samples.size(); i++)
    {
      double tmp_dist = (Eigen::Map<Eigen::VectorXf>(
          space_->samples_[space_->volumes_[volume_index].reach_samples[i]].q,
          space_->dimension_) - q_eigen).norm();
      if (tmp_dist < dist)
      {
        dist = tmp_dist;
        sample_index = space_->volumes_[volume_index].reach_samples[i];
      }
    }
    return true;
  }
}

