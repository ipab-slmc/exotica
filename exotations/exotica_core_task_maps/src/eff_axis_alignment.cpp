//
// Copyright (c) 2018, Wolfgang Merkt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <eigen_conversions/eigen_kdl.h>

#include <exotica_core/server.h>
#include <exotica_core_task_maps/eff_axis_alignment.h>

REGISTER_TASKMAP_TYPE("EffAxisAlignment", exotica::EffAxisAlignment);

namespace exotica
{
void EffAxisAlignment::Initialize()
{
    N = scene_->GetKinematicTree().GetNumControlledJoints();

    n_frames_ = parameters_.EndEffector.size();
    if (debug_) HIGHLIGHT_NAMED("EffAxisAlignment", "Number of EndEffectors: " << n_frames_);
    axis_.resize(3, n_frames_);
    dir_.resize(3, n_frames_);

    frames_.resize(2 * n_frames_);
    for (int i = 0; i < n_frames_; ++i)
    {
        FrameWithAxisAndDirectionInitializer frame(parameters_.EndEffector[i]);
        axis_.col(i) = frame.Axis.normalized();
        dir_.col(i) = frame.Direction.normalized();

        frames_[i + n_frames_] = frames_[i];
        tf::vectorEigenToKDL(axis_.col(i), frames_[i + n_frames_].frame_A_offset.p);
    }

    if (debug_)
    {
        for (int i = 0; i < n_frames_; ++i)
        {
            HIGHLIGHT_NAMED("EffAxisAlignment",
                            "Frame " << frames_[i].frame_A_link_name << ":"
                                     << "\tAxis=" << axis_.col(i).transpose()
                                     << "\tDirection=" << dir_.col(i).transpose());
        }
    }

    if (Server::IsRos())
    {
        pub_debug_ = Server::Advertise<visualization_msgs::MarkerArray>(object_name_ + "/debug", 1, true);
        msg_debug_.markers.reserve(n_frames_ * 2);
        for (int i = 0; i < n_frames_; ++i)
        {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.frame_locked = true;
            marker.header.frame_id = "exotica/" + scene_->GetKinematicTree().GetRootFrameName();
            marker.scale.x = 0.025;  // Shaft diameter
            marker.scale.y = 0.05;   // Head diameter
            marker.scale.z = 0.05;   // Head length
            marker.ns = frames_[i].frame_A_link_name;
            marker.pose.orientation.w = 1.0;
            marker.points.resize(2);

            // Current: red
            {
                marker.color = GetColor(1., 0., 0., 0.5);
                marker.id = i;
                msg_debug_.markers.emplace_back(marker);
            }

            // Target: green
            {
                marker.color = GetColor(0., 1., 0., 0.5);
                marker.id = i + n_frames_;
                msg_debug_.markers.emplace_back(marker);
            }
        }

        // Clear pre-existing markers
        visualization_msgs::MarkerArray msg;
        msg.markers.resize(1);
        msg.markers[0].action = 3;  // DELETE_ALL
        pub_debug_.publish(msg);
    }
}

void EffAxisAlignment::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void EffAxisAlignment::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi)
{
    if (phi.rows() != n_frames_) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_frames_; ++i)
    {
        link_position_in_base_ = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        link_axis_position_in_base_ = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i + n_frames_).p.data);

        Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
    }
}

void EffAxisAlignment::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != n_frames_) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != n_frames_ || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < n_frames_; ++i)
    {
        link_position_in_base_ = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        link_axis_position_in_base_ = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i + n_frames_).p.data);

        const Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        const Eigen::MatrixXd axisInBaseJacobian = kinematics[0].jacobian[i + n_frames_].data.topRows<3>() - kinematics[0].jacobian[i].data.topRows<3>();

        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
        jacobian.row(i) = dir_.col(i).transpose() * axisInBaseJacobian;

        if (Server::IsRos() && debug_)
        {
            constexpr double arrow_length = 0.25;
            // Current - red
            msg_debug_.markers[i].points[0].x = link_position_in_base_.x();
            msg_debug_.markers[i].points[0].y = link_position_in_base_.y();
            msg_debug_.markers[i].points[0].z = link_position_in_base_.z();
            msg_debug_.markers[i].points[1].x = link_position_in_base_.x() + arrow_length * axisInBase.x();
            msg_debug_.markers[i].points[1].y = link_position_in_base_.y() + arrow_length * axisInBase.y();
            msg_debug_.markers[i].points[1].z = link_position_in_base_.z() + arrow_length * axisInBase.z();

            // Target - green
            msg_debug_.markers[i + n_frames_].points[0].x = link_position_in_base_.x();
            msg_debug_.markers[i + n_frames_].points[0].y = link_position_in_base_.y();
            msg_debug_.markers[i + n_frames_].points[0].z = link_position_in_base_.z();
            msg_debug_.markers[i + n_frames_].points[1].x = link_position_in_base_.x() + arrow_length * dir_.col(i).x();
            msg_debug_.markers[i + n_frames_].points[1].y = link_position_in_base_.y() + arrow_length * dir_.col(i).y();
            msg_debug_.markers[i + n_frames_].points[1].z = link_position_in_base_.z() + arrow_length * dir_.col(i).z();

            pub_debug_.publish(msg_debug_);
        }
    }
}

int EffAxisAlignment::TaskSpaceDim()
{
    return n_frames_;
}

Eigen::Vector3d EffAxisAlignment::GetDirection(const std::string& frame_name) const
{
    for (int i = 0; i < n_frames_; ++i)
    {
        if (frames_[i].frame_A_link_name == frame_name)
        {
            return dir_.col(i);
        }
    }
    ThrowPretty("Direction for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::SetDirection(const std::string& frame_name, const Eigen::Vector3d& dir_in)
{
    for (int i = 0; i < n_frames_; ++i)
    {
        if (frames_[i].frame_A_link_name == frame_name)
        {
            dir_.col(i) = dir_in.normalized();
            return;
        }
    }
    ThrowPretty("Could not find frame with name " << frame_name << ".");
}

Eigen::Vector3d EffAxisAlignment::GetAxis(const std::string& frame_name) const
{
    for (int i = 0; i < n_frames_; ++i)
    {
        if (frames_[i].frame_A_link_name == frame_name)
        {
            return axis_.col(i);
        }
    }
    ThrowPretty("Axis for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::SetAxis(const std::string& frame_name, const Eigen::Vector3d& axis_in)
{
    for (int i = 0; i < n_frames_; ++i)
    {
        if (frames_[i].frame_A_link_name == frame_name)
        {
            axis_.col(i) = axis_in.normalized();
            tf::vectorEigenToKDL(axis_.col(i), frames_[i + n_frames_].frame_A_offset.p);
            return;
        }
    }
    ThrowPretty("Could not find frame with name " << frame_name << ".");
}
}  // namespace exotica
