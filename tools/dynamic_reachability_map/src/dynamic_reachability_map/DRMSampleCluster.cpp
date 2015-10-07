/*
 * DRMSampleCluster.cpp
 *
 *  Created on: 22 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMSampleCluster.h"
#include "dynamic_reachability_map/DRMSampler.h"
namespace dynamic_reachability_map
{
  DRMClusterParam::DRMClusterParam()
      : min_num_clusters(1), small_cluster_cutoff(0.02), max_joint_step(
          M_PI / 8), tolerance(1e-5), algorithm("MeanShift")
  {

  }

  DRMSampleCluster::DRMSampleCluster()
  {
    algorithms_["MeanShift"] = boost::bind(
        &DRMSampleCluster::meanShiftClustering, this, _1);
  }

  DRMSampleCluster::~DRMSampleCluster()
  {

  }

  bool DRMSampleCluster::startClustering(DRMSpace_ptr &space,
      DRMClusterParam &param)
  {
    if (algorithms_.find(param.algorithm) == algorithms_.end())
    {
      ROS_ERROR_STREAM("Unknown clustering algorithm ["<<param.algorithm<<"]");
      std::string tmp = "Known algorithms:";
      for (auto &it : algorithms_)
        tmp += " [" + it.first + "]";
      ROS_INFO_STREAM(tmp);
      return false;
    }
    space_ = space;
    var_index_ = space_->getGroup()->getVariableIndexList();
    algorithms_.at(param.algorithm)(param);

    //Process occupation for clusters
//  processOccupation();
    space->clear();
    dynamic_reachability_map::DRMSampler drms(space->thread_size_);
    std::vector<std::vector<double>> samples;
    for (unsigned int s = 0; s < clusters_.size(); s++)
    {
      for (unsigned int i = 0; i < clusters_[s].size(); i++)
      {
        std::vector<double> tmp(space->getDimension());
        for (int n = 0; n < space->getDimension(); n++)
          tmp[n] = clusters_[s][i].center(n);
        samples.push_back(tmp);
      }
    }
    drms.startSampling(space, clusters_.size(), samples);
    return true;
  }

  void DRMSampleCluster::processOccupation()
  {
    std::map<std::string, unsigned int> volume_map_;
    moveit_msgs::PlanningSceneWorld world_msg;
    for (unsigned int i = 0; i < space_->getSpaceSize(); i++)
    {
      moveit_msgs::CollisionObject volume_obj;
      volume_obj.header.frame_id =
          space_->ps_->getRobotModel()->getRootLinkName();
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
    space_->ps_->processPlanningSceneWorldMsg(world_msg);
    space_->clear();
    space_->initialiseSamples(clusters_cnt_);

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = space_->getSpaceSize();
    unsigned long int tmp_sample = 0;
    for (unsigned int i = 0; i < space_->space_size_; i++)
    {
      for (unsigned int j = 0; j < clusters_[i].size(); j++)
      {
        ROS_INFO_STREAM(
            "Processing cluster occupation "<<tmp_sample<<"/"<<clusters_cnt_);
        for (int n = 0; n < var_index_.size(); n++)
          space_->ps_->getCurrentStateNonConst().setVariablePosition(
              var_index_[n], clusters_[i][j].center(n));
        space_->ps_->getCurrentStateNonConst().update(true);
        tf::poseEigenToMsg(
            space_->ps_->getCurrentStateNonConst().getGlobalLinkTransform(
                space_->eff_), space_->getSampleNonConst(tmp_sample).effpose);

        space_->qEigen2Array(clusters_[i][j].center,
            space_->getSampleNonConst(tmp_sample).q);
        unsigned int tmp_check = 0;
        space_->getVolumeIndex(
            space_->getSampleNonConst(tmp_sample).effpose.position, tmp_check);
        space_->getSampleNonConst(tmp_sample).eff_index = tmp_check;
        space_->atNonConst(tmp_check).reach_samples.push_back(tmp_sample);

        std::map<int, bool> checked;
        collision_detection::CollisionResult result;
        space_->ps_->checkCollision(request, result);
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
              //This means the robot is in self collision, which should not happen. But just in case
              continue;
            }
            if (checked.find(tmp) == checked.end())
            {
              space_->atNonConst(tmp).occup_samples.push_back(tmp_sample);
              checked[tmp] = true;
            }
          }
        }

        tmp_sample++;
      }
    }
  }

  void DRMSampleCluster::meanShiftClustering(DRMClusterParam &param)
  {
    ROS_WARN_STREAM_NAMED("DRM MeanShift",
        "Minimum number of clusters per volume "<<param.min_num_clusters);
    ROS_WARN_STREAM_NAMED("DRM MeanShift",
        "Small clusters cutoff factor "<<param.small_cluster_cutoff);
    ROS_WARN_STREAM_NAMED("DRM MeanShift",
        "Maximum joint step "<<param.max_joint_step);
    sample_clustered_.resize(space_->sample_size_);
    for (unsigned long int i = 0; i < space_->sample_size_; i++)
      sample_clustered_[i] = false;

    clusters_.resize(space_->space_size_);
    unsigned int thread_size = space_->thread_size_;
    std::vector<boost::thread*> th(thread_size);

    unsigned int tmp = space_->space_size_ / thread_size;
    for (unsigned int i = 0; i < thread_size; ++i)
    {
      std::pair<unsigned int, unsigned int> volumes(i * tmp,
          i == thread_size - 1 ? space_->space_size_ : (i + 1) * tmp);
      th[i] = new boost::thread(
          boost::bind(&DRMSampleCluster::meanShiftThreadFn, this, i, volumes,
              param));
    }
    for (unsigned int i = 0; i < thread_size; ++i)
    {
      th[i]->join();
      delete th[i];
    }

    clusters_cnt_ = 0;
    for (unsigned int i = 0; i < space_->space_size_; i++)
      clusters_cnt_ += clusters_[i].size();
    ROS_WARN_STREAM(
        "Clustering finished !!!!!! "<<space_->sample_size_<<" are clustered into "<<clusters_cnt_);
  }

  void DRMSampleCluster::meanShiftThreadFn(int id,
      std::pair<unsigned int, unsigned int> &volumes, DRMClusterParam &param)
  {
    ROS_INFO_STREAM_NAMED("MeanShift",
        "Thread "<<id<<" start. Task volumes ["<<volumes.first<<" - "<<volumes.second<<").");
    for (unsigned int i = volumes.first; i < volumes.second; i++)
    {
      meanShiftClusterVolume(id, i, param);
    }
    ROS_WARN_STREAM_NAMED("MeanShift", "Thread "<<id<<" finished.");
  }

  void DRMSampleCluster::meanShiftClusterVolume(int thread_id,
      unsigned int index, DRMClusterParam &param)
  {
    //Finally, let's do the clustering
    unsigned int sample_size = space_->volumes_[index].reach_samples.size();
    if (sample_size == 0) return;
    //No need for clustering
    if (sample_size <= param.min_num_clusters)
    {
      clusters_[index].clear();
      for (unsigned long int i = 0; i < sample_size; i++)
      {
        Cluster tmp_cluster;
        space_->qArray2Eigen(
            space_->samples_[space_->volumes_[index].reach_samples[i]].q,
            tmp_cluster.center);
        tmp_cluster.samples.push_back(space_->volumes_[index].reach_samples[i]);
        clusters_[index].push_back(tmp_cluster);
        sample_clustered_[space_->volumes_[index].reach_samples[i]] = true;
      }

    }
//Clustering is needed
    else
    {
      VolumeClusters clusters;
      std::vector<Eigen::VectorXi> votes(0);
      std::vector<unsigned long int> samples(sample_size);
      std::vector<bool> open(sample_size);
      for (unsigned int i = 0; i < sample_size; i++)
      {
        samples[i] = space_->volumes_[index].reach_samples[i];
        open[i] = true;
      }
      Eigen::VectorXf old_mean(space_->dimension_);
      unsigned int cnt = sample_size;
      while (cnt > 0)
      {
        unsigned int tmp_index, start_index;
        while (true)
        {
          tmp_index = rand() % sample_size;
          if (!open[tmp_index]) continue;
          start_index = samples[tmp_index];
          break;
        }

        space_->qArray2Eigen(space_->samples_[start_index].q, old_mean);
        unsigned int iteration = 0;
        while (iteration < sample_size)
        {
          Eigen::VectorXf my_mean = Eigen::VectorXf::Zero(space_->dimension_);
          Eigen::VectorXi this_votes = Eigen::VectorXi::Zero(sample_size);
          std::vector<unsigned long int> my_members;
          my_members.reserve(sample_size);
          std::vector<unsigned int> closed;
          for (unsigned int i = 0; i < samples.size(); i++)
          {
            double max_dist = (old_mean
                - Eigen::Map<Eigen::VectorXf>(space_->samples_[samples[i]].q,
                    space_->dimension_)).norm(); //cwiseAbs().maxCoeff();
            if (max_dist < sqrt(space_->getDimension()) * param.max_joint_step)
            {
              this_votes(i) = this_votes(i) + 1;
              my_members.push_back(samples[i]);
              open[i] = false;
              my_mean += Eigen::Map<Eigen::VectorXf>(
                  space_->samples_[samples[i]].q, space_->dimension_);
            }
          }

          my_mean /= my_members.size();
          if ((my_mean - old_mean).norm() < param.tolerance)
          {
            bool new_cluster = true;
            for (auto &it : clusters)
            {
              if ((my_mean - it.second.center).norm()
                  < sqrt(space_->dimension_) * param.max_joint_step / 2)
              {
                new_cluster = false;
                it.second.center = (it.second.center + my_mean) / 2;
                votes[it.first] += this_votes;
                break;
              }
            }
            if (new_cluster)
            {
              Cluster new_cluster;
              new_cluster.center = my_mean;
              clusters[clusters.size()] = new_cluster;
              votes.push_back(this_votes);
            }
            break;
          }
          old_mean = my_mean;
          iteration++;
        }

        cnt = 0;
        for (int i = 0; i < open.size(); i++)
          if (open[i]) cnt++;
      }

      Eigen::MatrixXi votes_mat(clusters.size(), sample_size);
      for (unsigned int i = 0; i < clusters.size(); i++)
        votes_mat.row(i) = votes[i].transpose();
      std::vector<unsigned int> max_votes(sample_size);
      Eigen::MatrixXi votes_T = votes_mat.transpose();
      for (int i = 0; i < votes_T.rows(); i++)
      {
        Eigen::VectorXi::Index max_index;
        Eigen::VectorXi(votes_T.row(i)).maxCoeff(&max_index);
        clusters[max_index].samples.push_back(
            space_->volumes_[index].reach_samples[i]);
        sample_clustered_[space_->volumes_[index].reach_samples[i]] = true;
      }

      clusters_[index].clear();
      for (unsigned int i = 0; i < clusters.size(); i++)
      {
        if (clusters.size() <= param.min_num_clusters
            || clusters[i].samples.size()
                > param.small_cluster_cutoff * sample_size)
        {
          for (int n = 0; n < var_index_.size(); n++)
            space_->ps_->getCurrentStateNonConst().setVariablePosition(
                var_index_[n], clusters[i].center(n));
          space_->ps_->getCurrentStateNonConst().update(true);
          geometry_msgs::Pose tmppose;
          tf::poseEigenToMsg(
              space_->ps_->getCurrentStateNonConst().getGlobalLinkTransform(
                  space_->eff_), tmppose);
          unsigned int tmp_check = 0;
          if (space_->getVolumeIndex(tmppose.position, tmp_check))
            clusters_[index].push_back(clusters[i]);
        }
      }

      if (thread_id == 3)
      {
        ROS_INFO_STREAM(
            "Volume "<<index<<" has "<<sample_size<<" samples, "<< clusters.size()<<" clustered into "<<clusters_[index].size()<<" clusters. (cutoff "<<param.small_cluster_cutoff*sample_size<<")");
      }
    }
  }
}

