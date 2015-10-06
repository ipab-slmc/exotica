/*
 * Conversions.cpp
 *
 *  Created on: 22 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/Conversions.h"

namespace dynamic_reachability_map
{
std::vector<std::string> getStringVector(const std::string &s)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, ' '))
    elems.push_back(item);
  return elems;
}
void kdl2Pose(const KDL::Frame & kdl, geometry_msgs::Pose &pose)
{
  pose.position.x = kdl.p.x();
  pose.position.y = kdl.p.y();
  pose.position.z = kdl.p.z();
  kdl.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

KDL::Frame point2KDL(const geometry_msgs::Point &point)
{
  return KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(point.x, point.y, point.z));

}
KDL::Frame pose2KDL(const geometry_msgs::Pose &pose)
{
  return KDL::Frame(
      KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
      KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

double KDLDist2D(const KDL::Vector &v1, const KDL::Vector &v2)
{
  return (KDL::Vector2(v1.data[0], v1.data[1]) - KDL::Vector2(v2.data[0], v2.data[1])).Norm();
}
}
