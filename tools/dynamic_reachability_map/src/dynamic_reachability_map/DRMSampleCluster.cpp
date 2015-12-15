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
      : min_num_clusters(1), small_cluster_cutoff(0.0), max_joint_step(0.4), tolerance(
          1e-5), algorithm("MeanShift")
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
//    space->clear();
//    dynamic_reachability_map::DRMSampler drms(space->thread_size_);
//    std::vector<std::vector<double>> samples;
//    for (unsigned int s = 0; s < clusters_.size(); s++)
//    {
//      for (unsigned int i = 0; i < clusters_[s].size(); i++)
//      {
//        std::vector<double> tmp(space->getDimension());
//        for (int n = 0; n < space->getDimension(); n++)
//          tmp[n] = clusters_[s][i].center(n);
//        samples.push_back(tmp);
//      }
//    }
//    drms.startSampling(space, clusters_.size(), samples);
    return true;
  }

  bool DRMSampleCluster::saveClusters(const std::string &path)
  {
    std::ofstream cluster_file;
    cluster_file.open(path + "/Clusters.txt");
    for (unsigned int i = 0; i < space_->getSpaceSize(); i++)
    {
      cluster_file << space_->at(i).reach_clusters.size() << " ";
      for (unsigned int j = 0; j < space_->at(i).reach_clusters.size(); j++)
      {
        cluster_file << space_->at(i).reach_clusters[j].size() << " ";
        for (unsigned int k = 0; k < space_->at(i).reach_clusters[j].size();
            k++)
          cluster_file << space_->at(i).reach_clusters[j][k] << " ";
      }
      cluster_file << std::endl;
    }
    ROS_INFO_STREAM("Write clusters finished");
    cluster_file.close();
    return true;
  }

  bool DRMSampleCluster::loadClusters(const std::string &path,
      DRMSpace_ptr &space)
  {
    std::ifstream cluster_file;
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(path.c_str())) == NULL)
    {
      ROS_ERROR_STREAM("Result directory "<<path<<" does not exist");
      return false;
    }
    std::string file_name = "Clusters.txt";
    bool found = false;
    while ((dirp = readdir(dp)) != NULL)
    {
      if (dirp->d_name == file_name)
      {
        found = true;
        break;
      }
    }
    int cluster_cnt = 0;
    if (!found)
    {
      for (int i = 0; i < space->space_size_; i++)
      {
        space->volumes_[i].reach_clusters.resize(
            space->volumes_[i].reach_samples.size());
        cluster_cnt += space->volumes_[i].reach_clusters.size();
        for (int j = 0; j < space->volumes_[i].reach_clusters.size(); j++)
        {
          space->volumes_[i].reach_clusters[j].resize(1);
          space->volumes_[i].reach_clusters[j][0] =
              space->volumes_[i].reach_samples[j];
        }
      }
    }
    else
    {
      cluster_file.open(path + "/Clusters.txt");
      std::string line;
      for (int i = 0; i < space->space_size_; i++)
      {
        getline(cluster_file, line);
        std::vector<std::string> clusters = getStringVector(line);
        space->volumes_[i].reach_clusters.resize(std::stoi(clusters[0]));
        cluster_cnt += space->volumes_[i].reach_clusters.size();
        int cnt = 1;
        for (int j = 0; j < space->volumes_[i].reach_clusters.size(); j++)
        {
          int tmp = std::stoi(clusters[cnt]);
          space->volumes_[i].reach_clusters[j].resize(tmp);
          cnt++;
          for (int k = 0; k < tmp; k++)
          {
            space->volumes_[i].reach_clusters[j][k] =
                (unsigned long int) std::stoi(clusters[cnt]);
            space->samples_[space->volumes_[i].reach_clusters[j][k]].cluster =
                j;
            cnt++;
          }
        }
      }
    }
    ROS_INFO_STREAM(
        "DRM space contains "<<space->getSampleSize()<<" samples, "<<cluster_cnt<<" clusters");
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
    unsigned int thread_size = 8;
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
    int sample_size = space_->volumes_[index].reach_samples.size();
    if (sample_size == 0) return;
    //No need for clustering
    if (sample_size <= param.min_num_clusters)
    {
      clusters_[index].clear();
      for (unsigned long int i = 0; i < sample_size; i++)
      {
        Cluster tmp_cluster;
        tmp_cluster.samples.push_back(space_->volumes_[index].reach_samples[i]);
        clusters_[index].push_back(tmp_cluster);
        sample_clustered_[space_->volumes_[index].reach_samples[i]] = true;
      }

    }
//Clustering is needed
    else
    {
      if (thread_id == 0)
      {
        ROS_INFO_STREAM(
            "Clustering volume "<<index<<"/"<<space_->space_size_<<" with "<<sample_size<<" samples");
      }
      VolumeClusters clusters;
      std::vector<Eigen::VectorXi> votes(0);
      std::vector<unsigned long int> samples(sample_size);
      std::vector<bool> open(sample_size);
      for (int i = 0; i < sample_size; i++)
      {
        samples[i] = space_->volumes_[index].reach_samples[i];
        open[i] = true;
      }
      Eigen::VectorXf old_mean(space_->dimension_);
      unsigned int cnt = sample_size;
      while (cnt > 0)
      {
        unsigned long int start_index;
        int tmp_index;
        while (true)
        {
          tmp_index = rand() % sample_size;
          if (!open[tmp_index]) continue;
          start_index = samples[tmp_index];

          break;
        }
        double base_w = 1;
        double eff_w = 2;
        double q_w = 1;
        double q_w2 = 1;
        for (int j = 7; j < space_->dimension_; j++)
          old_mean(j) = q_w * space_->samples_[start_index].q[j];
        old_mean(7) *= q_w2;
        old_mean(8) *= q_w2;
        old_mean(9) *= q_w2;
        old_mean(11) *= q_w2;
        old_mean(13) *= q_w2;

        old_mean(0) = base_w * space_->samples_[start_index].q[0];
        old_mean(1) = base_w * space_->samples_[start_index].q[1];
        old_mean(2) = base_w * space_->samples_[start_index].q[2];
        old_mean(3) = eff_w
            * space_->samples_[start_index].effpose.orientation.x;
        old_mean(4) = eff_w
            * space_->samples_[start_index].effpose.orientation.y;
        old_mean(5) = eff_w
            * space_->samples_[start_index].effpose.orientation.z;
        old_mean(6) = eff_w
            * space_->samples_[start_index].effpose.orientation.w;

        while (true)
        {
          Eigen::VectorXf my_mean = Eigen::VectorXf::Zero(space_->dimension_);
          Eigen::VectorXi this_votes = Eigen::VectorXi::Zero(sample_size);
          std::vector<unsigned long int> my_members;
          my_members.reserve(sample_size);

          for (int i = 0; i < sample_size; i++)
          {
            Eigen::VectorXf tmp(space_->dimension_);
            for (int j = 7; j < space_->dimension_; j++)
              tmp(j) = q_w * space_->samples_[samples[i]].q[j];
            tmp(7) *= q_w2;
            tmp(8) *= q_w2;
            tmp(9) *= q_w2;
            tmp(11) *= q_w2;
            tmp(13) *= q_w2;
            tmp(0) = base_w * space_->samples_[samples[i]].q[0];
            tmp(1) = base_w * space_->samples_[samples[i]].q[1];
            tmp(2) = base_w * space_->samples_[samples[i]].q[2];
            tmp(3) = eff_w * space_->samples_[samples[i]].effpose.orientation.x;
            tmp(4) = eff_w * space_->samples_[samples[i]].effpose.orientation.y;
            tmp(5) = eff_w * space_->samples_[samples[i]].effpose.orientation.z;
            tmp(6) = eff_w * space_->samples_[samples[i]].effpose.orientation.w;
            if (i == tmp_index)
            {
              this_votes(i) = this_votes(i) + 1;
              my_members.push_back(i);
              my_mean += tmp;
            }
            else
            {
              double max_dist = (old_mean - tmp).norm(); //cwiseAbs().maxCoeff();
              if (max_dist < sqrt(space_->dimension_) * param.max_joint_step)
              {
                this_votes(i) = this_votes(i) + 1;
                my_members.push_back(i);
                my_mean += tmp;
              }
            }
          }
          my_mean /= my_members.size();
          if ((my_mean - old_mean).norm() <= param.tolerance)
          {
            bool new_cluster = true;
            int cluster_index = -1;
            for (auto &it : clusters)
            {
              cluster_index = it.first;
              if ((my_mean - it.second.center).norm()
                  < sqrt(space_->dimension_) * param.max_joint_step / 2)
              {
                new_cluster = false;
                it.second.center = (it.second.center + my_mean) / 2;
                votes[cluster_index] = votes[cluster_index] + this_votes;
                break;
              }
            }
            if (new_cluster)
            {
              cluster_index = clusters.size();
              Cluster new_cluster;
              new_cluster.center = my_mean;
              clusters[cluster_index] = new_cluster;
              votes.push_back(this_votes);
              votes[cluster_index] = this_votes;
            }
            for (int i = 0; i < sample_size; i++)
              if (votes[cluster_index](i) > 0) open[i] = false;
            break;
          }
          old_mean = my_mean;
        }

        cnt = 0;
        for (int i = 0; i < open.size(); i++)
          if (open[i]) cnt++;
      }
      space_->volumes_[index].reach_clusters.resize(clusters.size());
      for (int i = 0; i < samples.size(); i++)
      {
        int max_index = -1;
        int max = -1;
        for (int j = 0; j < clusters.size(); j++)
        {
          int tmp = votes[j](i);
          if (tmp > max)
          {
            max = tmp;
            max_index = j;
          }
        }
        if (max == 0)
        ROS_ERROR_STREAM("Volume "<<index<<" sample "<<i<<" not clustered");
        space_->volumes_[index].reach_clusters[max_index].push_back(
            space_->volumes_[index].reach_samples[i]);
        sample_clustered_[space_->volumes_[index].reach_samples[i]] = true;
      }
      if (thread_id == 0)
      {
        ROS_INFO_STREAM(
            "Volume "<<index<<" has "<<sample_size<<" samples, "<< clusters.size()<<" clusters");
      }
    }
  }
}

