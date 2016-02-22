/*
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
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
      || !nh.getParam("config", config_name))
  {
    INDICATE_FAILURE
    return 0;
  }
  if (ser.initialise(config_name, solver_name, problem_name, ""))
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
