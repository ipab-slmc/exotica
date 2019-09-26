//
// Copyright (c) 2019, Christopher E. Mower
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
#include <exotica_core_task_maps/eff_box.h>

REGISTER_TASKMAP_TYPE("EffBox", exotica::EffBox);

namespace exotica
{
void EffBox::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_effs_; ++i)
    {
        // Setup
        const int eff_id = 3 * i;

        // Compute phi
        phi.segment(eff_id, 3) = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data) - eff_upper_.segment<3>(eff_id);
        phi.segment(eff_id + three_times_n_effs_, 3) = eff_lower_.segment<3>(eff_id) - Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
    }

    if (debug_ && Server::IsRos()) PublishObjectsAsMarkerArray();
}

void EffBox::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != TaskSpaceDim() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < n_effs_; ++i)
    {
        // Setup
        const int eff_id = 3 * i;

        // Compute phi and jacobian
        phi.segment(eff_id, 3) = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data) - eff_upper_.segment<3>(eff_id);
        phi.segment(eff_id + three_times_n_effs_, 3) = eff_lower_.segment<3>(eff_id) - Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        jacobian.middleRows(eff_id, 3) = kinematics[0].jacobian(i).data.topRows<3>();
        jacobian.middleRows(eff_id + three_times_n_effs_, 3) = -kinematics[0].jacobian(i).data.topRows<3>();
    }

    if (debug_ && Server::IsRos()) PublishObjectsAsMarkerArray();
}

Eigen::Vector3d EffBox::GetLowerLimit(const int eff_id) const
{
    if (eff_id < 0 || eff_id >= n_effs_) ThrowNamed("Given eff_id (" << eff_id << ") is out of range [0, " << n_effs_ << ")!");
    return eff_lower_.segment<3>(3 * eff_id);
}

Eigen::Vector3d EffBox::GetUpperLimit(const int eff_id) const
{
    if (eff_id < 0 || eff_id >= n_effs_) ThrowNamed("Given eff_id (" << eff_id << ") is out of range [0, " << n_effs_ << ")!");
    return eff_upper_.segment<3>(3 * eff_id);
}

void EffBox::PublishObjectsAsMarkerArray()
{
    const ros::Time t = ros::Time::now();
    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(n_effs_);
    for (int i = 0; i < n_effs_; ++i)
    {
        const int eff_id = 3 * i;
        visualization_msgs::Marker m;
        m.header.stamp = t;
        std::string frame_name;
        if (frames_[i].frame_B_link_name == "")
        {
            frame_name = scene_->GetRootFrameName();
        }
        else
        {
            frame_name = frames_[i].frame_B_link_name;
        }
        m.header.frame_id = "exotica/" + frame_name;
        m.id = i;
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;
        m.scale.x = eff_upper_(eff_id) - eff_lower_(eff_id);
        m.scale.y = eff_upper_(eff_id + 1) - eff_lower_(eff_id + 1);
        m.scale.z = eff_upper_(eff_id + 2) - eff_lower_(eff_id + 2);
        m.pose.position.x = (eff_upper_(eff_id) + eff_lower_(eff_id)) / 2.0;
        m.pose.position.y = (eff_upper_(eff_id + 1) + eff_lower_(eff_id + 1)) / 2.0;
        m.pose.position.z = (eff_upper_(eff_id + 2) + eff_lower_(eff_id + 2)) / 2.0;
        m.pose.orientation.w = 1.0;
        m.color.r = 1.0;
        m.color.a = 0.25;
        ma.markers.emplace_back(m);
    }
    pub_markers_.publish(ma);
}

void EffBox::Instantiate(const EffBoxInitializer& init)
{
    parameters_ = init;
    n_effs_ = frames_.size();
    three_times_n_effs_ = 3 * n_effs_;

    // Populate eff_upper_ and eff_lower_
    eff_upper_.resize(3 * n_effs_, 1);
    eff_lower_.resize(3 * n_effs_, 1);

    for (int i = 0; i < n_effs_; ++i)
    {
        const int eff_id = 3 * i;

        // Initialize frame and check user input
        FrameWithBoxLimitsInitializer frame(parameters_.EndEffector[i]);
        if (frame.XLim[0] > frame.XLim[1]) ThrowPretty("Specify XLim using lower then upper for end-effector " << i << ".");
        if (frame.YLim[0] > frame.YLim[1]) ThrowPretty("Specify YLim using lower then upper for end-effector " << i << ".");
        if (frame.ZLim[0] > frame.ZLim[1]) ThrowPretty("Specify ZLim using lower then upper for end-effector " << i << ".");

        // Set upper and lower limits
        eff_upper_[eff_id] = frame.XLim[1];
        eff_upper_[eff_id + 1] = frame.YLim[1];
        eff_upper_[eff_id + 2] = frame.ZLim[1];

        eff_lower_[eff_id] = frame.XLim[0];
        eff_lower_[eff_id + 1] = frame.YLim[0];
        eff_lower_[eff_id + 2] = frame.ZLim[0];
    }

    if (debug_ && Server::IsRos())
    {
        pub_markers_ = Server::Advertise<visualization_msgs::MarkerArray>("eff_box_objects", 1, true);
        visualization_msgs::Marker md;  // delete previous markers
        md.action = 3;                  // DELETEALL
        visualization_msgs::MarkerArray ma;
        ma.markers.reserve(1);
        ma.markers.emplace_back(md);
        pub_markers_.publish(ma);
    }
}

int EffBox::TaskSpaceDim()
{
    return 6 * n_effs_;
}
}  // namespace exotica
