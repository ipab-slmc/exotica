/*
 * DRMHelpers.h
 *
 *  Created on: 5 Nov 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMHELPERS_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMHELPERS_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace dynamic_reachability_map
{
  struct SpaceBounds
  {
      SpaceBounds();
      SpaceBounds(double xlow, double xup, double ylow, double yup, double zlow,
          double zup);

      bool isValid();

      bool inBounds(const geometry_msgs::Point &p);
      bool inBounds(const Eigen::Affine3d &p);

      void print();

      double x_low;
      double x_upper;
      double y_low;
      double y_upper;
      double z_low;
      double z_upper;
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMHELPERS_H_ */
