/*
 * DRMOctreeSpace.h
 *
 *  Created on: 5 Nov 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMOCTREESPACE_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMOCTREESPACE_H_

#include "dynamic_reachability_map/Conversions.h"
#include <ros/ros.h>

namespace dynamic_reachability_map
{
  struct Sample
  {
      Sample();

      ~Sample();

      void invalidate();

      float* q;
      geometry_msgs::Pose effpose;
      unsigned int eff_index;
      bool isValid;
  };

  class DRMOctreeSpace
  {
    public:
      DRMOctreeSpace();
      ~DRMOctreeSpace();
    private:
      unsigned int depth_;
      double resolution_;

  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_SRC_DYNAMIC_REACHABILITY_MAP_DRMOCTREESPACE_H_ */
