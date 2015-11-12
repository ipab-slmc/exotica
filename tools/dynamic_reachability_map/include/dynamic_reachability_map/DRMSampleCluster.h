/*
 * DRMSampleCluster.h
 *
 *  Created on: 22 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMSAMPLECLUSTER_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMSAMPLECLUSTER_H_

#include "dynamic_reachability_map/DRMSpace.h"

namespace dynamic_reachability_map
{
  struct DRMClusterParam
  {
      DRMClusterParam();
      unsigned int min_num_clusters;
      double small_cluster_cutoff;
      double max_joint_step;
      double tolerance;
      std::string algorithm;
  };

  struct Cluster
  {
      Eigen::VectorXf center;
      std::vector<unsigned long int> samples;
  };
  typedef std::map<int, Cluster> VolumeClusters;
  typedef std::vector<std::vector<Cluster> > Clusters;

  class DRMSampleCluster
  {
    public:
      DRMSampleCluster();
      ~DRMSampleCluster();
      bool startClustering(DRMSpace_ptr &space, DRMClusterParam &param);
      bool loadClusters(const std::string &path, DRMSpace_ptr &space);
      bool saveClusters(const std::string &path);
      Clusters clusters_;
      unsigned long int clusters_cnt_;
    private:
      void meanShiftClustering(DRMClusterParam &param);
      void meanShiftThreadFn(int id,
          std::pair<unsigned int, unsigned int> &volumes,
          DRMClusterParam &param);
      void meanShiftClusterVolume(int thread_id, unsigned int index,
          DRMClusterParam &param);
      void processOccupation();
      std::map<std::string, boost::function<void(DRMClusterParam &param)> > algorithms_;
      DRMSpace_ptr space_;
      std::vector<bool> sample_clustered_;
      std::vector<int> var_index_;
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMSAMPLECLUSTER_H_ */
