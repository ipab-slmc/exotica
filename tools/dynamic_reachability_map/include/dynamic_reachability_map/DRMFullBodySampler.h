/*
 * DRMFullBodySampler.h
 *
 *  Created on: 20 Nov 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMFULLBODYSAMPLER_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMFULLBODYSAMPLER_H_

#include "dynamic_reachability_map/DRMSpace.h"
#include "dynamic_reachability_map/DRMSampler.h"
#include <exotica/EXOTica.hpp>
#include <drake/RigidBodyIK.h>
#include <drake/RigidBodyManipulator.h>
#include <drake/RigidBodyConstraint.h>
#include <drake/IKoptions.h>
namespace dynamic_reachability_map
{

  class DRMFullBodySampler: public DRMSampler
  {
    public:
      DRMFullBodySampler(int nt);
      virtual ~DRMFullBodySampler();
      bool initialise(const std::string &urdf);
    private:
      unsigned long int setToNextState(int id, Eigen::Affine3d &effpose,
          unsigned int &volume_index);
      void drakeIK(const Eigen::VectorXd &start, Eigen::VectorXd &sol, int &info, std::vector<std::string> &infeasible);
      RigidBodyManipulator* model_;
      std::vector<RigidBodyConstraint*> constraints_;
      std::vector<std::string> joints_;
      std::vector<int> joints_idx_;
      Eigen::VectorXd reach_start_;
      IKoptions* ik_options_;

      ros::NodeHandle nh_;
      ros::Publisher state_pub_;
      ros::Publisher cell_pub_;
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMFULLBODYSAMPLER_H_ */
