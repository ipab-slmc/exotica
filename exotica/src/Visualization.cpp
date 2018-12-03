/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2016, Wolfgang Merkt
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

#include <exotica/Visualization.h>

namespace exotica
{
Visualization::Visualization(Scene_ptr scene) : scene_(scene)
{
    HIGHLIGHT_NAMED("Visualization", "Initialising visualizer");
    Initialize();
}
Visualization::~Visualization() = default;

void Visualization::Initialize()
{
    if (Server::isRos())
    {
        trajectory_pub_ = Server::advertise<moveit_msgs::DisplayTrajectory>(scene_->getName() + (scene_->getName().empty() ? "" : "/") + "Trajectory", 1, true);
    }
}

void Visualization::displayTrajectory(Eigen::MatrixXdRefConst trajectory)
{
    if (!Server::isRos()) return;

    if (trajectory.cols() != scene_->getSolver().getNumControlledJoints())
        throw_pretty("Number of DoFs in trajectory does not match number of controlled joints of robot.");

    moveit_msgs::DisplayTrajectory traj_msg;
    traj_msg.model_id = scene_->getSolver().getRobotModel()->getName();

    // TODO:
    //   http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
    // TODO: Support for fixed and floating base! - multi_dof_joint_trajectory
    // TODO: Support for uncontrolled joints: trajectory_start

    traj_msg.trajectory.resize(1);
    traj_msg.trajectory[0].joint_trajectory.header.frame_id = scene_->getRootFrameName();
    traj_msg.trajectory[0].joint_trajectory.header.stamp = ros::Time::now();
    traj_msg.trajectory[0].joint_trajectory.joint_names = scene_->getSolver().getJointNames();
    traj_msg.trajectory[0].joint_trajectory.points.resize(trajectory.rows());

    for (int i = 0; i < trajectory.rows(); i++)
    {
        traj_msg.trajectory[0].joint_trajectory.points[i].positions.resize(trajectory.cols());
        for (int n = 0; n < trajectory.cols(); n++)
        {
            traj_msg.trajectory[0].joint_trajectory.points[i].positions[n] = trajectory(i, n);
        }
    }
    trajectory_pub_.publish(traj_msg);
}
}