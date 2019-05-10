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

#include <exotica_core_task_maps/eff_axis_alignment.h>

REGISTER_TASKMAP_TYPE("EffAxisAlignment", exotica::EffAxisAlignment);

namespace exotica
{
EffAxisAlignment::EffAxisAlignment() = default;
EffAxisAlignment::~EffAxisAlignment() = default;

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
}

void EffAxisAlignment::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void EffAxisAlignment::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != n_frames_) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_frames_; ++i)
    {
        tf::vectorKDLToEigen(kinematics[0].Phi(i).p, link_position_in_base_);
        tf::vectorKDLToEigen(kinematics[0].Phi(i + n_frames_).p, link_axis_position_in_base_);

        Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
    }
}

void EffAxisAlignment::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != n_frames_) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != n_frames_ || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < n_frames_; ++i)
    {
        tf::vectorKDLToEigen(kinematics[0].Phi(i).p, link_position_in_base_);
        tf::vectorKDLToEigen(kinematics[0].Phi(i + n_frames_).p, link_axis_position_in_base_);

        const Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        const Eigen::MatrixXd axisInBaseJacobian = kinematics[0].jacobian[i + n_frames_].data.block(0, 0, 3, N) - kinematics[0].jacobian[i].data.block(0, 0, 3, N);

        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
        jacobian.row(i) = dir_.col(i).transpose() * axisInBaseJacobian;
    }
}

int EffAxisAlignment::TaskSpaceDim()
{
    return n_frames_;
}

Eigen::Vector3d EffAxisAlignment::GetDirection(const std::string& frame_name)
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

Eigen::Vector3d EffAxisAlignment::GetAxis(const std::string& frame_name)
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
