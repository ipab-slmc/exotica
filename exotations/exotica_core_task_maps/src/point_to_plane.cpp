//
// Copyright (c) 2018-2020, Wolfgang Merkt, University of Oxford
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

#include <exotica_core/server.h>
#include <exotica_core_task_maps/point_to_plane.h>
#include <kdl_conversions/kdl_msg.h>

REGISTER_TASKMAP_TYPE("PointToPlane", exotica::PointToPlane);

namespace exotica
{
void PointToPlane::Instantiate(const PointToPlaneInitializer& init)
{
    if (debug_ && Server::IsRos())
    {
        debug_pub_ = Server::Advertise<visualization_msgs::MarkerArray>(object_name_ + "/planes", 1, true);
    }
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = kinematics[0].Phi(i).p.data[2];
    }

    if (debug_ && Server::IsRos()) PublishDebug();
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = kinematics[0].Phi(i).p.data[2];
        jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
    }

    if (debug_ && Server::IsRos()) PublishDebug();
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = kinematics[0].Phi(i).p.data[2];
        jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
        hessian(i).block(0, 0, jacobian.cols(), jacobian.cols()) = kinematics[0].hessian[i](2);
    }
}

int PointToPlane::TaskSpaceDim()
{
    return kinematics[0].Phi.rows();
}

void PointToPlane::PublishDebug()
{
    visualization_msgs::MarkerArray msg;

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        visualization_msgs::Marker plane;

        plane.header.frame_id = frames_[i].frame_B_link_name == "" ? "exotica/" + scene_->GetRootFrameName() : frames_[i].frame_B_link_name;
        plane.ns = frames_[i].frame_A_link_name;
        plane.id = i;
        plane.type = plane.CUBE;
        plane.action = plane.ADD;
        plane.frame_locked = true;
        plane.color.g = 1.0f;
        plane.color.a = 0.8f;
        tf::poseKDLToMsg(frames_[i].frame_B_offset, plane.pose);

        plane.scale.x = 10.f;
        plane.scale.y = 10.f;
        plane.scale.z = 0.01f;
        plane.pose.position.z -= (plane.scale.z / 2);

        msg.markers.push_back(plane);
    }

    debug_pub_.publish(msg);
}
}  // namespace exotica
