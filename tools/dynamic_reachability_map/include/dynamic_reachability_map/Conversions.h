/*
 * Conversions.h
 *
 *  Created on: 22 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_CONVERSIONS_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_CONVERSIONS_H_

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>

namespace dynamic_reachability_map
{
std::vector<std::string> getStringVector(const std::string &s);
void kdl2Pose(const KDL::Frame & kdl, geometry_msgs::Pose &pose);
KDL::Frame point2KDL(const geometry_msgs::Point &point);
KDL::Frame pose2KDL(const geometry_msgs::Pose &pose);
double KDLDist2D(const KDL::Vector &v1, const KDL::Vector &v2);
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_CONVERSIONS_H_ */
