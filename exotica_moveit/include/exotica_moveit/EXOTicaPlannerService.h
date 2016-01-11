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

#ifndef EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_
#define EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_

#include <aico/AICOsolver.h>
#include <ompl_solver/OMPLsolver.h>
#include <ik_solver/ik_solver.h>
#include <exotica/Initialiser.h>
#include <generic/Identity.h>
#include <exotica_moveit/ExoticaPlanningAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

namespace exotica
{
  class EXOTicaPlannerService
  {
    public:
      EXOTicaPlannerService();
      ~EXOTicaPlannerService();

      bool initialise(const std::string & config, const std::string & solver,
          const std::string & problem, const std::string & group);

      /*
       * \brief	Solve function
       * @param	goal	Planning goal and constraints
       */
      bool solve(const exotica_moveit::ExoticaPlanningGoalConstPtr & goal);
      bool initialised_;
    private:
      ///	The ROS node handle
      ros::NodeHandle nh_;

      ///	ROS action service
      actionlib::SimpleActionServer<exotica_moveit::ExoticaPlanningAction> as_;

      ///	Action result
      exotica_moveit::ExoticaPlanningResult res_;

      ///	Action feedback
      exotica_moveit::ExoticaPlanningFeedback fb_;

      ///	EXOTica server
      exotica::Server_ptr server_;

      ///	Pointer to selected solver
      exotica::MotionSolver_ptr solver_;

      ///	Pointer to selected problem
      exotica::PlanningProblem_ptr problem_;
      boost::shared_ptr<exotica::Identity> goal_bias_map_;
      boost::shared_ptr<exotica::Identity> goal_map_;

      ///	ROS service
      ros::ServiceServer service_;

      ///	Moveit planning scene
      moveit_msgs::PlanningScenePtr scene_;

  };
}

#endif /* EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_ */
