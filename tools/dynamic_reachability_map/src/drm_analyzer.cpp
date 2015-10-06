/*
 * drm_analyzer.cpp
 *
 *  Created on: 3 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/drm_analyzer.h";

namespace dynamic_reachability_map
{
DRMAnalyzer::DRMAnalyzer() :
    nh_("~")
{

}

DRMAnalyzer::~DRMAnalyzer()
{

}

bool DRMAnalyzer::initialise()
{
  std::string model, drm_model, path, eff, group;
  if (!nh_.hasParam("RobotModel"))
    model = "robot_description";
  else
    nh_.getParam("RobotModel", model);
  if (!nh_.getParam("DRMModel", drm_model))
  {
    ROS_ERROR("Dynamic reachability map urdf model not defined [rosparam: DRMModel]");
    return false;
  }
  if (!nh_.getParam("DRMPath", path))
  {
    ROS_ERROR("Dynamic reachability map files not defined [rosparam: DRMPath]");
    return false;
  }
  if (!nh_.getParam("EndEffector", eff))
  {
    ROS_ERROR("EndEffector link not defined [rosparam: EndEffector]");
    return false;
  }
  if (!nh_.getParam("Group", group))
  {
    ROS_ERROR("Group name not defined [rosparam: Group]");
    return false;
  }

  robot_model_loader::RobotModelLoader robot_model_loader(model);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  if (!drm_.createRobotSampler(kinematic_model, eff, group))
    return false;
  if (!drm_.loadSpace(path))
  {
    ROS_ERROR_STREAM("Can not load dynamic reacability map from "<<path);
    return false;
  }

  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "~");
  ros::AsyncSpinner sp(1);
  dynamic_reachability_map::DRMAnalyzer drma;
  if (!drma.initialise())
  {
    ROS_ERROR("ERROR initialise dynamic reachability map");
    return 0;
  }
  sp.start();
  ros::waitForShutdown();
  return 0;
}

