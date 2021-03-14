//
// Copyright (c) 2021, University of Oxford
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
#include <exotica_core_task_maps/distance_to_line_2d.h>

REGISTER_TASKMAP_TYPE("DistanceToLine2D", exotica::DistanceToLine2D);

namespace exotica
{
void DistanceToLine2D::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void DistanceToLine2D::Initialize()
{
    if (frames_.size() != 3)
    {
        ThrowPretty("This task map requires three frames to initialise: The first two define the line, the third is the query frame.");
    }

    if (Server::IsRos())
    {
        pub_debug_ = Server::Advertise<visualization_msgs::MarkerArray>(object_name_ + "/debug", 1, true);

        debug_marker_array_msg_.markers.resize(4);

        // First = P1
        // Second = P2
        // Third = P3
        for (std::size_t i = 0; i < 4; ++i)
        {
            debug_marker_array_msg_.markers[i].type = visualization_msgs::Marker::SPHERE;
            debug_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
            debug_marker_array_msg_.markers[i].id = i;
            debug_marker_array_msg_.markers[i].color.a = .7f;
            debug_marker_array_msg_.markers[i].color.r = 0.5f;
            debug_marker_array_msg_.markers[i].color.g = 0.f;
            debug_marker_array_msg_.markers[i].color.b = 0.f;
            debug_marker_array_msg_.markers[i].scale.x = debug_marker_array_msg_.markers[i].scale.y = debug_marker_array_msg_.markers[i].scale.z = .02f;
            debug_marker_array_msg_.markers[i].pose.orientation.w = 1.0;

            debug_marker_array_msg_.markers[i].header.frame_id = "exotica/" + scene_->GetRootFrameName();
        }
        // Query point in blue:
        debug_marker_array_msg_.markers[2].color.r = 0.f;
        debug_marker_array_msg_.markers[2].color.b = 0.5f;

        // Fourth = Line
        debug_marker_array_msg_.markers[3].type = visualization_msgs::Marker::LINE_LIST;
    }
}

void DistanceToLine2D::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi)
{
    if (phi.rows() != 1) ThrowNamed("Wrong size of Phi!");

    const Eigen::Vector2d P1(kinematics[0].Phi(0).p.data[0], kinematics[0].Phi(0).p.data[1]);
    const Eigen::Vector2d P2(kinematics[0].Phi(1).p.data[0], kinematics[0].Phi(1).p.data[1]);
    const Eigen::Vector2d P3(kinematics[0].Phi(2).p.data[0], kinematics[0].Phi(2).p.data[1]);
    PointToLineDistance(P1, P2, P3, phi(0));

    if (Server::IsRos() && debug_)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            debug_marker_array_msg_.markers[i].pose.position.x = kinematics[0].Phi(i).p.data[0];
            debug_marker_array_msg_.markers[i].pose.position.y = kinematics[0].Phi(i).p.data[1];
            debug_marker_array_msg_.markers[i].pose.position.z = 0.0;
        }

        // Line strip
        debug_marker_array_msg_.markers[3].points.resize(2);
        for (std::size_t i = 0; i < 2; ++i)
        {
            debug_marker_array_msg_.markers[3].points[i].x = kinematics[0].Phi(i).p.data[0];
            debug_marker_array_msg_.markers[3].points[i].y = kinematics[0].Phi(i).p.data[1];
            debug_marker_array_msg_.markers[3].points[i].z = 0.0;
        }

        pub_debug_.publish(debug_marker_array_msg_);
    }
}

void DistanceToLine2D::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != 1) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != 1 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    const Eigen::Vector2d P1(kinematics[0].Phi(0).p.data[0], kinematics[0].Phi(0).p.data[1]);
    const Eigen::Vector2d P2(kinematics[0].Phi(1).p.data[0], kinematics[0].Phi(1).p.data[1]);
    const Eigen::Vector2d P3(kinematics[0].Phi(2).p.data[0], kinematics[0].Phi(2).p.data[1]);
    PointToLineDistance(P1, P2, P3, phi(0));
    PointToLineDistanceDerivative(P1, P2, P3, kinematics[0].jacobian[0].data, kinematics[0].jacobian[1].data, kinematics[0].jacobian[2].data, jacobian);
}

int DistanceToLine2D::TaskSpaceDim()
{
    return 1;
}

}  // namespace exotica
