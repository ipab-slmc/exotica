/*
 * AICOexample.cpp
 *
 *  Created on: 30 Apr 2014
 *      Author: Vladimir Ivan
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

#include "aico_solver_demo/AICOexample.h"

namespace exotica
{

  AICOexample::AICOexample()
      : nh_("~"), nhg_()
  {
    std::string resource_path = ros::package::getPath("aico").append(
        "/resources/");
    ROS_INFO_STREAM("Loaded path: " << resource_path);

    // Declarations
    MotionSolver_ptr sol;
    Server_ptr ser;
    PlanningProblem_ptr prob;

    // Get config file path, problem name and solver name from launch file
    std::string problem_name, solver_name, config_name;
    nh_.getParam("config", config_name);
    nh_.getParam("problem", problem_name);
    nh_.getParam("solver", solver_name);
    ROS_INFO_STREAM(
        "Config: "<<config_name<<"\nSolver: "<<solver_name<<"\nProblem: "<<problem_name);

    // Initialise and solve
      Initialiser::Instance()->initialise(config_name, ser, sol, prob, problem_name, solver_name);
      // Cast the generic solver instance into AICO solver
      AICOsolver_ptr solAICO = boost::static_pointer_cast<AICOsolver>(sol);

      // Assign the problem to the solver
      sol->specifyProblem(prob);

      // If necessary, modify the problem after calling sol->specifyProblem()
      // e.g. set different rho:
      for (int t = 0; t < solAICO->getProblem()->getT(); t++)
      {
        // This sets the precision of all time steps BUT the last one to zero
        // This means we only aim to minimize the task cost in the last time step
        // The rest of the trajectory minimizes the control cost
        solAICO->rhos.at(t).setZero();
      }

      // Create the initial configuration
      Eigen::VectorXd q = Eigen::VectorXd::Zero(
          prob->scenes_.begin()->second->getNumJoints());
      Eigen::MatrixXd solution;
      ROS_INFO_STREAM("Calling solve()");
      {
        ros::WallTime start_time = ros::WallTime::now();
        // Solve the problem using the AICO solver
        try
        {
          solAICO->Solve(q, solution);
          double time = ros::Duration(
              (ros::WallTime::now() - start_time).toSec()).toSec();
          ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving ("<<time<<"s)");
          ROS_INFO_STREAM_THROTTLE(0.5,
              "Solution "<<solution.row(solution.rows()-1));
          solAICO->saveCosts(std::string("costs.txt"));

          // Publish the states to rviz
          jointStatePublisher_ = nhg_.advertise<sensor_msgs::JointState>(
              "/joint_states", 1);
          sensor_msgs::JointState jnt;
          jnt.position.resize(solution.cols());
          jnt.name = prob->scenes_.begin()->second->getSolver().getJointNames();
          ros::Rate loop_rate(1.0 / solAICO->getProblem()->getTau());
          int t = 0;
          ROS_INFO_STREAM_THROTTLE(0.5, "Publishing states to rviz ...");
          while (ros::ok())
          {
            jnt.header.stamp = ros::Time::now();
            jnt.header.seq++;
            for (int j = 0; j < solution.cols(); j++)
              jnt.position[j] = solution(t, j);
            jointStatePublisher_.publish(jnt);

            t = t + 1 >= solution.rows() ? 0 : t + 1;
            ros::spinOnce();
            loop_rate.sleep();
          }
        }
        catch (SolveException e)
        {
          double time = ros::Duration(
              (ros::WallTime::now() - start_time).toSec()).toSec();
          ROS_INFO_STREAM_THROTTLE(0.5,
              e.what()<<" ("<<time<<"s)");
        }
      }


  }

  AICOexample::~AICOexample()
  {
    // TODO Auto-generated destructor stub
  }

} /* namespace exotica */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AICOexample");
  ROS_INFO_STREAM("Started");
  exotica::AICOexample ex;
}
