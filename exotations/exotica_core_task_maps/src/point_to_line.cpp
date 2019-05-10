//
// Copyright (c) 2018, University of Edinburgh
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
#include <exotica_core_task_maps/point_to_line.h>

REGISTER_TASKMAP_TYPE("PointToLine", exotica::PointToLine);

namespace exotica
{
PointToLine::PointToLine() = default;
PointToLine::~PointToLine() = default;

Eigen::Vector3d PointToLine::Direction(const Eigen::Vector3d &point)
{
    // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    // let:
    //      s: start of line
    //      e: end of line
    //      p: point
    // then the point on vector v = e-s that is closest to p is vp = s + t*(e-s) with
    // t = -((s-p)*(e-s)) / (|e-s|^2)    (* denotes the dot product)
    if (debug_) HIGHLIGHT_NAMED("P2L", "\e[4m" << link_name_ << "\e[0m");
    if (debug_) HIGHLIGHT_NAMED("P2L", "p " << point.transpose());
    if (debug_) HIGHLIGHT_NAMED("P2L", "ls " << line_start_.transpose());
    if (debug_) HIGHLIGHT_NAMED("P2L", "le " << line_end_.transpose());
    double t = -(line_start_ - point).dot(line_) / line_.squaredNorm();
    std::stringstream ss;
    ss << "t " << t;
    if (!infinite_)
    {
        // clip to to range [0,1]
        t = std::min(std::max(0.0, t), 1.0);
        ss << ", clipped |t| " << t;
    }
    if (debug_) HIGHLIGHT_NAMED("P2L", ss.str());

    // vector from point 'p' to point 'vp' on line
    // vp = line_start_ + t * (line_end_-line_start_)
    const Eigen::Vector3d dv = line_start_ + (t * line_) - point;
    if (debug_) HIGHLIGHT_NAMED("P2L", "vp " << (line_start_ + (t * line_)).transpose());
    if (debug_) HIGHLIGHT_NAMED("P2L", "dv " << dv.transpose());
    return dv;
}

Eigen::Vector3d PointToLine::GetEndPoint()
{
    return line_end_;
}

void PointToLine::SetEndPoint(const Eigen::Vector3d &point)
{
    line_end_ = point;
    line_ = line_end_ - line_start_;
}

void PointToLine::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows() * 3) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        const Eigen::Vector3d p = line_start_ + Eigen::Map<const Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi.segment<3>(i * 3) = -Direction(p);
    }
}

void PointToLine::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * 3) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 3 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        // point in base frame
        const Eigen::Vector3d p = line_start_ + Eigen::Map<const Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        // direction from point to line
        const Eigen::Vector3d dv = Direction(p);
        phi.segment<3>(i * 3) = dv;

        if ((dv + p - line_start_).norm() < std::numeric_limits<double>::epsilon())
        {
            // clipped (t=0) case
            jacobian.middleRows<3>(i * 3) = -kinematics[0].jacobian[i].data.topRows<3>();
        }
        else
        {
            for (int j = 0; j < jacobian.cols(); ++j)
            {
                jacobian.middleRows<3>(i * 3).col(j) = kinematics[0].jacobian[i].data.topRows<3>().col(j).dot(line_ / line_.squaredNorm()) * line_ - kinematics[0].jacobian[i].data.topRows<3>().col(j);
            }
        }

        // visualisation of point, line and their distance
        if (visualize_ && Server::IsRos())
        {
            const ros::Time t = ros::Time::now();
            const std::string common_frame = "exotica/" + base_name_;
            visualization_msgs::MarkerArray ma;
            {
                // line in base frame
                visualization_msgs::Marker mc;
                mc.header.stamp = t;
                mc.frame_locked = true;
                mc.header.frame_id = common_frame;
                mc.ns = "cam/line/" + object_name_;
                mc.type = visualization_msgs::Marker::ARROW;
                mc.scale.x = 0.01;
                mc.scale.y = 0.01;
                mc.scale.z = 0.01;
                // line start
                geometry_msgs::Point pp;
                pp.x = line_start_.x();
                pp.y = line_start_.y();
                pp.z = line_start_.z();
                mc.points.push_back(pp);
                // line end
                const Eigen::Vector3d pe = p + dv;
                pp.x = pe.x();
                pp.y = pe.y();
                pp.z = pe.z();
                mc.points.push_back(pp);
                mc.color.r = 1;
                mc.color.g = 1;
                mc.color.b = 0;
                mc.color.a = 1;
                ma.markers.push_back(mc);
            }
            {
                // point in link frame
                visualization_msgs::Marker ml;
                ml.header.stamp = t;
                ml.frame_locked = true;
                ml.header.frame_id = common_frame;
                ml.ns = "lnk/point/" + object_name_;
                ml.type = visualization_msgs::Marker::SPHERE;
                ml.scale.x = 0.03;
                ml.scale.y = 0.03;
                ml.scale.z = 0.03;
                ml.color.r = 1;
                ml.color.g = 0;
                ml.color.b = 0;
                ml.color.a = 1;
                ml.pose.position.x = p.x();
                ml.pose.position.y = p.y();
                ml.pose.position.z = p.z();
                ma.markers.push_back(ml);
            }
            {
                // draw 'dv' starting at 'p' in base frame
                visualization_msgs::Marker mdv;
                mdv.header.stamp = t;
                mdv.frame_locked = true;
                mdv.header.frame_id = common_frame;
                mdv.ns = "dv/" + object_name_;
                mdv.type = visualization_msgs::Marker::ARROW;
                mdv.scale.x = 0.001;
                mdv.scale.y = 0.01;
                mdv.scale.z = 0.01;
                mdv.pose.position.x = p.x();
                mdv.pose.position.y = p.y();
                mdv.pose.position.z = p.z();
                mdv.points.push_back(geometry_msgs::Point());
                geometry_msgs::Point pdv;
                pdv.x = dv.x();
                pdv.y = dv.y();
                pdv.z = dv.z();
                mdv.points.push_back(pdv);
                mdv.color.r = 0;
                mdv.color.g = 1;
                mdv.color.b = 0;
                mdv.color.a = 0.5;
                ma.markers.push_back(mdv);
            }
            pub_marker_.publish(ma);
            {
                ma.markers.clear();
                visualization_msgs::Marker mt;
                mt.header.stamp = t;
                mt.frame_locked = true;
                mt.header.frame_id = common_frame;
                mt.ns = "lnk/label/" + object_name_;
                mt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                mt.text = link_name_;
                mt.pose.position.x = p.x();
                mt.pose.position.y = p.y();
                mt.pose.position.z = p.z();
                mt.scale.x = 0.05;
                mt.scale.y = 0.05;
                mt.scale.z = 0.05;
                mt.color.r = 1;
                mt.color.g = 1;
                mt.color.b = 1;
                mt.color.a = 1;
                ma.markers.push_back(mt);
                pub_marker_label_.publish(ma);
            }
        }
    }
}

void PointToLine::Instantiate(const PointToLineInitializer &init)
{
    link_name_ = frames_[0].frame_A_link_name;
    base_name_ = frames_[0].frame_B_link_name;

    line_start_ = Eigen::Map<Eigen::Vector3d>(frames_[0].frame_B_offset.p.data);
    line_end_ = init.EndPoint;

    line_ = line_end_ - line_start_;
    infinite_ = init.Infinite;

    visualize_ = init.Visualise;

    if (visualize_ && Server::IsRos())
    {
        pub_marker_ = Server::Advertise<visualization_msgs::MarkerArray>("p2l", 1, true);
        pub_marker_label_ = Server::Advertise<visualization_msgs::MarkerArray>("p2l_label", 1, true);
        // delete previous markers
        visualization_msgs::Marker md;
        md.action = 3;  // DELETEALL
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(md);
        pub_marker_.publish(ma);
        pub_marker_label_.publish(ma);
    }
}

int PointToLine::TaskSpaceDim()
{
    return kinematics[0].Phi.rows() * 3;
}
}  // namespace exotica
