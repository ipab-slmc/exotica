/*
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


#include "ik_solver_demo/node.h"

using namespace exotica;

IKSolverDemoNode::IKSolverDemoNode()
    : nh_("~"), nhg_()
{

  {
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
      // Assign the problem to the solver
      sol->specifyProblem(prob);
      // Create the initial configuration
      Eigen::VectorXd q = Eigen::VectorXd::Zero(
          prob->scenes_.begin()->second->getNumJoints());
      Eigen::MatrixXd solution;
      // Cast the generic solver instance into IK solver
      exotica::IKsolver_ptr solIK =
          boost::static_pointer_cast<exotica::IKsolver>(sol);
      ROS_INFO_STREAM("Calling solve() in an infinite loop");

      // Publish the states to rviz
      jointStatePublisher_ = nhg_.advertise<sensor_msgs::JointState>(
          "/joint_states", 1);
      sensor_msgs::JointState jnt;
      jnt.name = prob->scenes_.begin()->second->getSolver().getJointNames();
      jnt.position.resize(jnt.name.size());
      double t = 0.0;
      ros::Rate loop_rate(1000.0);
      ros::WallTime init_time = ros::WallTime::now();

      while (ros::ok())
      {
        ros::WallTime start_time = ros::WallTime::now();

        // Update the goal if necessary
        // e.g. figure eight
        t = ros::Duration((ros::WallTime::now() - init_time).toSec()).toSec();
        Eigen::VectorXd goal(6);
        goal << 0.6,
                -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                0.5 + sin(t * M_PI * 0.5) * 0.2,
                1.1,
                -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                0.5 + sin(t * M_PI * 0.5) * 0.2;
        solIK->setGoal("IKSolverDemoTask", goal, 0);

        // Solve the problem using the IK solver
        try
        {
          solIK->Solve(q, solution);
        }
        catch (SolveException e)
        {
          // Ignore failures
        }
        double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
        ROS_INFO_STREAM_THROTTLE(0.5,
          "Finished solving ("<<time<<"s, "<<solIK->getLastIteration()<<"), error: "<<solIK->error);
        q = solution.row(solution.rows() - 1);
        ROS_INFO_STREAM_THROTTLE(0.5, "Solution "<<solution);

        jnt.header.stamp = ros::Time::now();
        jnt.header.seq++;
        for (int j = 0; j < solution.cols(); j++)
        jnt.position[j] = q(j);
        jointStatePublisher_.publish(jnt);

        ros::spinOnce();
        loop_rate.sleep();
      }

    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "IKSolverDemoNode");
  ROS_INFO_STREAM("Started");
  IKSolverDemoNode ex;
  ros::spin();
}
