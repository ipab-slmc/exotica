/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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

#include <exotica/Exotica.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace exotica;

void run()
{
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));

    Initializer solver_initializer, problem_initializer;

    std::string file_name, solver_name, problem_name;
    Server::getParam("ConfigurationFile", file_name);
    Server::getParam("Solver", solver_name);
    Server::getParam("Problem", problem_name);

    XMLLoader::load(file_name, solver_initializer, problem_initializer, solver_name, problem_name);

    HIGHLIGHT_NAMED("XMLnode", "Loaded from XML");

    // Initialize

    PlanningProblem_ptr any_problem = Setup::createProblem(problem_initializer);
    MotionSolver_ptr any_solver = Setup::createSolver(solver_initializer);

    // Assign the problem to the solver
    any_solver->specifyProblem(any_problem);

    // If necessary, modify the problem after calling sol->specifyProblem()
    // e.g. set different rho:

    if (any_problem->type() == "exotica::UnconstrainedTimeIndexedProblem")
    {
        UnconstrainedTimeIndexedProblem_ptr problem = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(any_problem);
        for (int t = 0; t < problem->getT() - 1; t++)
        {
            // This sets the precision of all time steps BUT the last one to zero
            // This means we only aim to minimize the task cost in the last time step
            // The rest of the trajectory minimizes the control cost
            problem->setRho("Frame", 0.0, t);
        }
        problem->setRho("Frame", 1e3, 99);
    }

    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->N);
    Eigen::MatrixXd solution;
    ROS_INFO_STREAM("Calling solve()");
    {
        ros::WallTime start_time = ros::WallTime::now();
        // Solve the problem using the AICO solver
        any_solver->Solve(solution);
        double time = ros::Duration(
                          (ros::WallTime::now() - start_time).toSec())
                          .toSec();
        ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving (" << time << "s)");
        ROS_INFO_STREAM_THROTTLE(0.5, "Solution " << solution.row(solution.rows() - 1));

        ros::Rate loop_rate(50.0);
        int t = 0;
        ROS_INFO_STREAM_THROTTLE(0.5, "Publishing states to rviz ...");
        int PlaybackWaitInterval = 30;
        while (ros::ok())
        {
            int i = 1;
            if (t == 0 || t == solution.rows() - 1) i = PlaybackWaitInterval;
            while (i-- > 0)
            {
                any_problem->getScene()->Update(solution.row(t).transpose());
                any_problem->getScene()->getKinematicTree().publishFrames();

                ros::spinOnce();
                loop_rate.sleep();
            }
            t = t + 1 >= solution.rows() ? 0 : t + 1;
        }
    }

    // All classes will be destroyed at this point.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExoticaManualInitializationExampleNode");
    ROS_INFO_STREAM("Started");

    // Run demo code
    run();

    // Clean up
    // Run this only after all the exoica classes have been disposed of!
    Setup::Destroy();
}
