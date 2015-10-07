/*
 * DRMSampler.h
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSAMPLER_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSAMPLER_H_

#include "dynamic_reachability_map/DRMSpace.h"

namespace dynamic_reachability_map
{

  class DRMSampler
  {
    public:
      DRMSampler(int nt);
      ~DRMSampler();

      void startSampling(DRMSpace_ptr &space, unsigned long int sample_cnt,
          std::vector<std::vector<double>> &samples);
    private:
      void multiThreadSamplingFn(int id);
      std::vector<std::vector<double>> preProcessSamples(
          std::vector<std::vector<double>> &samples);
      unsigned long int setToNextState(int id, Eigen::Affine3d &effpose,
          unsigned int &volume_index);
      DRMSpace_ptr space_;
      std::vector<planning_scene::PlanningScenePtr> ps_;
      std::map<std::string, unsigned int> volume_map_;
      int thread_cnt_;
      unsigned long int sample_cnt_;
      std::vector<std::vector<double>> samples_;
      unsigned long int curr_cnt_;
      boost::mutex curr_cnt_lock;
      std::vector<std::vector<Volume>> tmp_volumes_;
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSAMPLER_H_ */
