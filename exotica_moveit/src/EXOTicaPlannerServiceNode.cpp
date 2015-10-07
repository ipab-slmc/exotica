/*
 * EXOTicaPlannerServiceNode.cpp
 *
 *  Created on: 23 Mar 2015
 *      Author: yiming
 */

#include <exotica_moveit/EXOTicaPlannerService.h>
#include <exotica/EXOTica.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exotica_planning_action_service");
  ros::NodeHandle nh("~");
  exotica::EXOTicaPlannerService ser;
  std::string problem_name;
  std::string solver_name;
  std::string config_name;
  std::string group_name;
  if (!nh.getParam("problem", problem_name)
      || !nh.getParam("solver", solver_name)
      || !nh.getParam("config", config_name)
      || !nh.getParam("planning_group", group_name))
  {
    INDICATE_FAILURE
    return 0;
  }
  if (ser.initialise(config_name, solver_name, problem_name, group_name))
  {
    HIGHLIGHT_NAMED("MoveitInterface", "Exotica Planning Action is ready");
    ros::spin();
    ros::waitForShutdown();
    HIGHLIGHT_NAMED("MoveitInterface", "Shutting Down Exotica Planning Service");
  }
  else
  {
    ERROR("Exotica Planning Action is NOT ready");
  }
  return 0;
}
