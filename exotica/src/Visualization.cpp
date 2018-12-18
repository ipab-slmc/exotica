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
    // TODO: Correctly handle dt for time_from_start
    // TODO:
    //   http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html

    if (!Server::isRos()) return;

    if (trajectory.cols() != scene_->getSolver().getNumControlledJoints())
        throw_pretty("Number of DoFs in trajectory does not match number of controlled joints of robot. Got " << trajectory.cols() << " but expected " << scene_->getSolver().getNumControlledJoints());

    moveit_msgs::DisplayTrajectory traj_msg;
    traj_msg.model_id = scene_->getSolver().getRobotModel()->getName();

    const int num_trajectory_points = trajectory.rows();

    traj_msg.trajectory.resize(1);
    traj_msg.trajectory[0].joint_trajectory.header.frame_id = scene_->getRootFrameName();
    traj_msg.trajectory[0].joint_trajectory.header.stamp = ros::Time::now();

    // Floating base support
    int base_offset = 0;
    switch (scene_->getSolver().getControlledBaseType())
    {
        case BASE_TYPE::FIXED:
            // HIGHLIGHT("Fixed base, no offset");
            break;
        case BASE_TYPE::PLANAR:
            // HIGHLIGHT("Planar, offset 3");
            base_offset = 3;
            break;
        case BASE_TYPE::FLOATING:
            // HIGHLIGHT("Floating, offset 6");
            base_offset = 6;
            break;
        default:
            throw_pretty("Unknown base type.");
    }

    // Insert floating base if not a fixed-base controlled robot
    if (scene_->getSolver().getControlledBaseType() != BASE_TYPE::FIXED)
    {
        traj_msg.trajectory[0].multi_dof_joint_trajectory.header.frame_id = scene_->getRootFrameName();
        traj_msg.trajectory[0].multi_dof_joint_trajectory.joint_names.push_back(scene_->getSolver().getRootJointName());
        traj_msg.trajectory[0].multi_dof_joint_trajectory.points.resize(num_trajectory_points);

        geometry_msgs::Transform base_transform;
        Eigen::Quaterniond quat;
        for (int i = 0; i < num_trajectory_points; i++)
        {
            base_transform.translation.x = trajectory(i, 0);
            base_transform.translation.y = trajectory(i, 1);

            if (scene_->getSolver().getControlledBaseType() == BASE_TYPE::FLOATING)
            {
                base_transform.translation.z = trajectory(i, 2);
                quat = Eigen::AngleAxisd(trajectory(i, 3), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(trajectory(i, 4), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(trajectory(i, 5), Eigen::Vector3d::UnitZ());
            }
            else if (scene_->getSolver().getControlledBaseType() == BASE_TYPE::PLANAR)
            {
                base_transform.translation.z = 0.0;  // TODO: Technically: trans_z
                quat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(trajectory(i, 2), Eigen::Vector3d::UnitZ());
            }

            base_transform.rotation.x = quat.x();
            base_transform.rotation.y = quat.y();
            base_transform.rotation.z = quat.z();
            base_transform.rotation.w = quat.w();

            traj_msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms.push_back(base_transform);
        }
    }

    // Handle unactuated joints, i.e., joints whose values are not part of the trajectory:
    // TODO: How to handle unactuated floating-base?
    traj_msg.trajectory_start.joint_state.header.frame_id = scene_->getRootFrameName();

    // Insert all starting joint states - including the floating base
    auto model_state_map = scene_->getSolver().getModelStateMap();
    for (const auto& pair : model_state_map)
    {
        // Only push back if not part of the floating base - i.e., the joint name includes the root frame name.
        if (pair.first.find(scene_->getRootFrameName()) == std::string::npos)
        {
            traj_msg.trajectory_start.joint_state.name.push_back(pair.first);
            traj_msg.trajectory_start.joint_state.position.push_back(pair.second);
        }
    }

    // Handle actuated joints - the joint names need to _exclude_ the base joints.
    const int num_actuated_joints_without_base = trajectory.cols() - base_offset;
    traj_msg.trajectory[0].joint_trajectory.points.resize(num_trajectory_points);
    traj_msg.trajectory[0].joint_trajectory.joint_names.resize(num_actuated_joints_without_base);
    for (int i = 0; i < num_actuated_joints_without_base; i++)
        traj_msg.trajectory[0].joint_trajectory.joint_names[i] = scene_->getSolver().getJointNames()[base_offset + i];

    // Insert actuated joints without base
    for (int i = 0; i < num_trajectory_points; i++)
    {
        traj_msg.trajectory[0].joint_trajectory.points[i].positions.resize(num_actuated_joints_without_base);
        for (int n = 0; n < num_actuated_joints_without_base; n++)
        {
            traj_msg.trajectory[0].joint_trajectory.points[i].positions[n] = trajectory(i, n + base_offset);
        }
    }

    trajectory_pub_.publish(traj_msg);
}
}