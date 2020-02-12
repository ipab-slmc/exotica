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

#include <iostream>
#include <kdl/path.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <string>

#include <exotica_core/tools.h>
#include <exotica_core/trajectory.h>

namespace exotica
{
Trajectory::Trajectory() : radius_(1.0), trajectory_(nullptr)
{
}

Trajectory::Trajectory(const std::string& data)
{
    std::istringstream ss(data);
    ss >> radius_;
    int n, m;
    ss >> n;
    ss >> m;
    data_.resize(n, m);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            double val;
            ss >> val;
            data_(i, j) = val;
        }
    }
    ConstructFromData(data_, radius_);
}

Trajectory::Trajectory(Eigen::MatrixXdRefConst data, double radius)
{
    ConstructFromData(data, radius);
}

KDL::Frame Trajectory::GetPosition(double t)
{
    return trajectory_->Pos(t);
}

KDL::Twist Trajectory::GetVelocity(double t)
{
    return trajectory_->Vel(t);
}

KDL::Twist Trajectory::GetAcceleration(double t)
{
    return trajectory_->Acc(t);
}

double Trajectory::GetDuration()
{
    return trajectory_->Duration();
}

Eigen::MatrixXd Trajectory::GetData()
{
    return data_;
}

double Trajectory::GetRadius()
{
    return radius_;
}

std::string Trajectory::ToString()
{
    std::ostringstream ss;
    ss << radius_ << "\n";
    ss << data_.rows() << " " << data_.cols() << "\n";
    ss << data_;
    return ss.str();
}

void Trajectory::ConstructFromData(Eigen::MatrixXdRefConst data, double radius)
{
    if (!(data.cols() == 4 || data.cols() == 7 || data.cols() == 8) || data.rows() < 2) ThrowPretty("Invalid trajectory data size!\nNeeds to contain 4, 7, or 8 columns and at least 2 rows.");
    trajectory_.reset(new KDL::Trajectory_Composite());
    for (int i = 0; i < data.rows() - 1; ++i)
    {
        KDL::Frame f1 = GetFrame(data.row(i).tail(data.cols() - 1).transpose());
        KDL::Frame f2 = GetFrame(data.row(i + 1).tail(data.cols() - 1).transpose());
        double dt = data(i + 1, 0) - data(i, 0);
        if (dt <= 0) ThrowPretty("Time indices must be monotonically increasing! " << i << " (" << dt << ")");
        if (KDL::Equal(f1, f2, 1e-6))
        {
            trajectory_->Add(new KDL::Trajectory_Stationary(dt, f1));
        }
        else
        {
            KDL::RotationalInterpolation_SingleAxis* rot = new KDL::RotationalInterpolation_SingleAxis();
            KDL::Path_Line* path = new KDL::Path_Line(f1, f2, rot, radius);
            KDL::VelocityProfile_Spline* vel = new KDL::VelocityProfile_Spline();
            KDL::Trajectory_Segment* seg = new KDL::Trajectory_Segment(path, vel, dt);
            trajectory_->Add(seg);
        }
    }
    data_ = data;
    radius_ = radius;
}
}  // namespace exotica
