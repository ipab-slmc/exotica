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
#include "dynamic_reachability_map/DRMTrajAction.h"

namespace dynamic_reachability_map
{
  struct AStarNode
  {
      AStarNode()
          : g(0.0), h(0.0), parent(-1), sample_index(-1)
      {

      }
      AStarNode(double g_, double h_)
          : g(g_), h(h_), parent(-1), sample_index(-1)
      {

      }
      double f()
      {
        return g + h;
      }
      double g;
      double h;
      int sample_index;
      int parent;
  };
  class DRM
  {
    public:
      DRM();
      virtual ~DRM();

      bool initialise(const std::string &drms_path,
          const robot_model::RobotModelConstPtr &model);

      const DRMSpace_ptr & space() const;
      DRMSpace_ptr & spaceNonConst();
      void resetDRM2FreeSpace();
      void updateOccupation(const std::map<unsigned int, bool> &occup_list);

      std::vector<std::pair<unsigned int, double> > getNeighborIndices(
          unsigned long int index, unsigned int depth = 1);
      dynamic_reachability_map::DRMResult getIKSolution(
          const dynamic_reachability_map::DRMGoalConstPtr &goal);
      dynamic_reachability_map::DRMTrajResult getTrajectory(
          const dynamic_reachability_map::DRMTrajGoalConstPtr &goal);
    private:
      bool getClosestSample(
          const dynamic_reachability_map::DRMGoalConstPtr &goal,
          std::vector<unsigned long int> &candidate_samples,
          unsigned long int &sample_index, bool use_invalids = true);
      bool getClosestSample(const exotica::Vector &q,
          unsigned long int &sample_index, unsigned int &space_index);
      bool getSampleReachIndex(const exotica::Vector &q,
          unsigned int &space_index);
      bool searchAStar(unsigned int start_space_index,
          unsigned int goal_space_index, std::vector<unsigned int> &space_path);

      double heuristicCost(unsigned int a, unsigned int b);
      DRMSpace_ptr space_;
      Eigen::MatrixXd W_;
      std::map<unsigned int, bool> invalid_volumes_;
      ros::NodeHandle nh_;
      ros::Publisher astar_pub_;
      visualization_msgs::Marker astar_mark_;
  };

  typedef boost::shared_ptr<DRM> DRM_ptr;

}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRM_H_ */
