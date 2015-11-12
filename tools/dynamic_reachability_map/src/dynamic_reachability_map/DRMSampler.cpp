/*
 * DRMSampler.cpp
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMSampler.h"

double dRand(double dMin, double dMax)
{
  double d = (double) rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

namespace dynamic_reachability_map
{

  DRMSampler::DRMSampler(int nt)
      : thread_cnt_(nt)
  {

  }

  DRMSampler::~DRMSampler()
  {
  }

  void DRMSampler::startSampling(DRMSpace_ptr &space,
      unsigned long int sample_cnt, std::vector<std::vector<double>> &samples)
  {
    space->clear();
    space->thread_size_ = thread_cnt_;
    space_ = space;

    ps_.resize(thread_cnt_);
    moveit_msgs::PlanningSceneWorld world_msg;
    for (unsigned int i = 0; i < space_->getSpaceSize(); i++)
    {
      moveit_msgs::CollisionObject volume_obj;
      volume_obj.header.frame_id =
          space->ps_->getRobotModel()->getRootLinkName();
      volume_obj.id = std::to_string(i);
      volume_map_[volume_obj.id] = i;
      shape_msgs::SolidPrimitive cell;
      cell.type = cell.BOX;
      cell.dimensions.resize(3);
      cell.dimensions[0] = cell.dimensions[1] = cell.dimensions[2] =
          space_->getResolution();
      geometry_msgs::Pose pose;
      pose.orientation.w = 1;
      pose.position = space_->at(i).center;
      volume_obj.primitives.push_back(cell);
      volume_obj.primitive_poses.push_back(pose);
      volume_obj.operation = volume_obj.APPEND;
      world_msg.collision_objects.push_back(volume_obj);
    }

    tmp_volumes_.resize(thread_cnt_);
    for (int t = 0; t < thread_cnt_; t++)
    {
      ps_[t].reset(
          new planning_scene::PlanningScene(space_->ps_->getRobotModel()));
      ps_[t]->processPlanningSceneWorldMsg(world_msg);
      tmp_volumes_[t].resize(space_->getSpaceSize());
    }
    if (samples.size() > 0)
    {
      samples_ = preProcessSamples(samples);
      sample_cnt_ = samples_.size();
    }
    else
      sample_cnt_ = sample_cnt;
    space->initialiseSamples(sample_cnt_);

    ROS_WARN_STREAM(
        "DRM Space: volume size="<<space->getSpaceSize()<<" resolution "<<space->getResolution()<<" required samples = "<<sample_cnt_);
    curr_cnt_ = 0;

    std::vector<boost::thread*> th(thread_cnt_);
    for (int i = 0; i < thread_cnt_; i++)
      th[i] = new boost::thread(
          boost::bind(&DRMSampler::multiThreadSamplingFn, this, i));
    for (unsigned int i = 0; i < thread_cnt_; ++i)
    {
      th[i]->join();
      delete th[i];
    }
    ROS_INFO("Sampling finished, combining threads' DRMs into central DRM");
    for (int t = 0; t < thread_cnt_; t++)
    {
      ROS_INFO_STREAM("Processing thread "<<t);
      for (unsigned int i = 0; i < space_->getSpaceSize(); i++)
      {
        for (unsigned int l = 0; l < tmp_volumes_[t][i].occup_samples.size();
            l++)
          space_->addOccupSample(i, tmp_volumes_[t][i].occup_samples[l]);
        for (unsigned int l = 0; l < tmp_volumes_[t][i].reach_samples.size();
            l++)
          space_->addReachSample(i, tmp_volumes_[t][i].reach_samples[l]);
        tmp_volumes_[t][i].occup_samples.clear();
        tmp_volumes_[t][i].reach_samples.clear();
      }
      tmp_volumes_[t].clear();
    }
    ROS_INFO("Sampling finished");
  }

  std::vector<std::vector<double>> DRMSampler::preProcessSamples(
      std::vector<std::vector<double>> &samples)
  {
    std::vector<std::vector<double>> ret;
    collision_detection::CollisionRequest self_req;
    collision_detection::CollisionResult self_res;
    unsigned int index = 0;
    Eigen::Affine3d effpose;
    for (unsigned long int i = 0; i < samples.size(); i++)
    {
      std::cout << "Pre-process samples: ("
          << (double) i / (double) samples.size() * 100.0 << "%).\r";
      std::cout.flush();
      for (int n = 0; n < space_->getDimension(); n++)
        space_->ps_->getCurrentStateNonConst().setVariablePosition(
            space_->var_index_[n], samples[i][n]);
      space_->ps_->getCurrentStateNonConst().update(true);

      effpose = space_->ps_->getCurrentStateNonConst().getGlobalLinkTransform(
          space_->eff_);
      if (!space_->getVolumeIndex(effpose, index)) continue;
      space_->ps_->checkSelfCollision(self_req, self_res);
      if (!self_res.collision) ret.push_back(samples[i]);
      self_res.clear();
    }
    ROS_INFO_STREAM(
        "Extracted "<<ret.size()<<" self-collision free samples from "<<samples.size()<<" samples");
    return ret;
  }

  unsigned long int DRMSampler::setToNextState(int id, Eigen::Affine3d &effpose,
      unsigned int &volume_index)
  {
    boost::mutex::scoped_lock(curr_cnt_lock);
    unsigned long int index = curr_cnt_;
    if (index == sample_cnt_) return index + 1;

    curr_cnt_++;

    if (samples_.size() > 0)
    {
      for (int i = 0; i < space_->getDimension(); i++)
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            space_->var_index_[i], samples_[index][i]);
      ps_[id]->getCurrentStateNonConst().update(true);
      effpose = ps_[id]->getCurrentStateNonConst().getGlobalLinkTransform(
          space_->eff_);
      space_->getVolumeIndex(effpose, volume_index);
    }
    else
    {
      collision_detection::CollisionRequest self_req;
      self_req.group_name = space_->group_->getName();
      collision_detection::CollisionResult self_res;
      while (true)
      {
        srand(time(NULL));
        ps_[id]->getCurrentStateNonConst().setToRandomPositions(space_->group_);
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/trans_x", dRand(-0.2, 0.2));
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/trans_y", dRand(-0.25, 0.25));
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/trans_z", dRand(-0.35, 0.15));
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/rot_x", 0);
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/rot_y", 0);
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/rot_z", 0);
        ps_[id]->getCurrentStateNonConst().setVariablePosition(
            "world_joint/rot_w", 1);
        ps_[id]->getCurrentStateNonConst().setVariablePosition("torsoPitch",
            dRand(-0.13, 0.5));
        ps_[id]->getCurrentStateNonConst().update(true);
        effpose = ps_[id]->getCurrentStateNonConst().getGlobalLinkTransform(
            space_->eff_);
        if (!space_->getVolumeIndex(effpose, volume_index)) continue;
        ps_[id]->checkSelfCollision(self_req, self_res);
        if (!self_res.collision)
        {
          self_res.clear();
          break;
        }
        self_res.clear();
      }
    }
    return index;
  }

  void DRMSampler::multiThreadSamplingFn(int id)
  {
    ROS_INFO_STREAM("Thread "<<id<<" started");
    unsigned int volume_index = 0;
    Eigen::Affine3d effpose;
    while (true)
    {
      unsigned long int sample_index = setToNextState(id, effpose,
          volume_index);
      if (sample_index >= sample_cnt_) break;
      if (id == 0)
      {
        std::cout << "Sampling process [" << id << "]: ("
            << (double) sample_index / (double) sample_cnt_ * 100.0 << "%).\r";
        std::cout.flush();
      }
      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = space_->getSpaceSize();
      request.group_name = space_->group_->getName();
      collision_detection::CollisionResult result;
      std::map<int, bool> checked;
      ps_[id]->checkCollision(request, result);
      if (result.collision)
      {
        for (auto &it : result.contacts)
        {
          unsigned int tmp;
          if (volume_map_.find(it.first.first) != volume_map_.end())
          {
            tmp = volume_map_.at(it.first.first);
          }
          else if (volume_map_.find(it.first.second) != volume_map_.end())
          {
            tmp = volume_map_.at(it.first.second);
          }
          else
          {
            ROS_ERROR_STREAM(
                "This should not happen! Contact between "<<it.first.first<<" and "<<it.first.second);
            continue;
          }
          if (checked.find(tmp) == checked.end())
          {
            tmp_volumes_[id][tmp].occup_samples.push_back(sample_index);
//          space_->addOccupSample(tmp, sample_index);
            checked[tmp] = true;
          }
        }
      }
      for (int i = 0; i < space_->var_index_.size(); i++)
        space_->samples_[sample_index].q[i] =
            ps_[id]->getCurrentState().getVariablePosition(
                space_->var_index_[i]);
      space_->samples_[sample_index].eff_index = volume_index;
      tf::poseEigenToMsg(effpose, space_->samples_[sample_index].effpose);
//    space_->addReachSample(volume_index, sample_index);
      tmp_volumes_[id][volume_index].reach_samples.push_back(sample_index);
    }
  }
}

