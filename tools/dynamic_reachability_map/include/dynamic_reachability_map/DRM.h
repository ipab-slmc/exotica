/*
 * DRM.h
 *
 *  Created on: 15 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRM_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRM_H_

#include "dynamic_reachability_map/DRMSpace.h"
#include "dynamic_reachability_map/DRMSpaceLoader.h"

#include "dynamic_reachability_map/DRMAction.h"

namespace dynamic_reachability_map
{
class DRM
{
public:
  DRM();
  virtual ~DRM();

  bool initialise(const std::string &drms_path, const robot_model::RobotModelConstPtr &model);

  const DRMSpace_ptr & space() const;
  DRMSpace_ptr & spaceNonConst();
  void resetDRM2FreeSpace();

  std::vector<std::pair<unsigned int, double> > getNeighborIndices(unsigned long int index, unsigned int depth);
  dynamic_reachability_map::DRMResult getIKSolution(const dynamic_reachability_map::DRMGoalConstPtr &goal);
  bool solve(const dynamic_reachability_map::DRMGoalConstPtr &goal, unsigned long int goal_sample_index,
             std::vector<unsigned long int> &path);
private:
  bool getClosestSample(const dynamic_reachability_map::DRMGoalConstPtr &goal,
                        std::vector<unsigned long int> &candidate_samples, unsigned long int &sample_index,
                        bool use_invalids = true);
  bool getClosestSample(const exotica::Vector &q, unsigned long int &sample_index);
  bool searchAStar(unsigned long int start_sample_index, unsigned long int goal_sample_index,
                   std::vector<unsigned long int> &path);

  double heuristicCost(unsigned long int a, unsigned long int b);
  DRMSpace_ptr space_;
  Eigen::MatrixXd W_;
};

typedef boost::shared_ptr<DRM> DRM_ptr;

}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRM_H_ */
