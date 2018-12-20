/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2018, Wolfgang Merkt
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

#include "exotica_core_task_maps/eff_axis_alignment.h"

REGISTER_TASKMAP_TYPE("EffAxisAlignment", exotica::EffAxisAlignment);

namespace exotica
{
EffAxisAlignment::EffAxisAlignment() = default;
EffAxisAlignment::~EffAxisAlignment() = default;

void EffAxisAlignment::Instantiate(EffAxisAlignmentInitializer& init)
{
    init_ = init;
}

void EffAxisAlignment::initialize()
{
    N = scene_->getKinematicTree().getNumControlledJoints();

    n_frames_ = init_.EndEffector.size();
    if (debug_) HIGHLIGHT_NAMED("EffAxisAlignment", "Number of EndEffectors: " << n_frames_);
    axis_.resize(3, n_frames_);
    dir_.resize(3, n_frames_);

    Frames.resize(2 * n_frames_);
    for (unsigned int i = 0; i < n_frames_; i++)
    {
        FrameWithAxisAndDirectionInitializer frame(init_.EndEffector[i]);
        axis_.col(i) = frame.Axis.normalized();
        dir_.col(i) = frame.Direction.normalized();

        Frames[i + n_frames_] = Frames[i];
        tf::vectorEigenToKDL(axis_.col(i), Frames[i + n_frames_].FrameAOffset.p);
    }

    if (debug_)
    {
        for (unsigned int i = 0; i < n_frames_; i++)
        {
            HIGHLIGHT_NAMED("EffAxisAlignment",
                            "Frame " << Frames[i].FrameALinkName << ":"
                                     << "\tAxis=" << axis_.col(i).transpose()
                                     << "\tDirection=" << dir_.col(i).transpose());
        }
    }
}

void EffAxisAlignment::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    initialize();
}

void EffAxisAlignment::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != n_frames_) throw_named("Wrong size of phi!");

    for (unsigned int i = 0; i < n_frames_; i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, link_position_in_base_);
        tf::vectorKDLToEigen(Kinematics[0].Phi(i + n_frames_).p, link_axis_position_in_base_);

        Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
    }
}

void EffAxisAlignment::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != n_frames_) throw_named("Wrong size of phi!");
    if (J.rows() != n_frames_ || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    for (unsigned int i = 0; i < n_frames_; i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, link_position_in_base_);
        tf::vectorKDLToEigen(Kinematics[0].Phi(i + n_frames_).p, link_axis_position_in_base_);

        Eigen::Vector3d axisInBase = link_axis_position_in_base_ - link_position_in_base_;
        Eigen::MatrixXd axisInBaseJacobian = Kinematics[0].J[i + n_frames_].data.block(0, 0, 3, N) - Kinematics[0].J[i].data.block(0, 0, 3, N);

        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
        J.row(i) = dir_.col(i).transpose() * axisInBaseJacobian;
    }
}

int EffAxisAlignment::taskSpaceDim()
{
    return n_frames_;
}

Eigen::Vector3d EffAxisAlignment::get_direction(const std::string& frame_name)
{
    for (unsigned int i = 0; i < n_frames_; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            return dir_.col(i);
        }
    }
    throw_pretty("Direction for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::set_direction(const std::string& frame_name, const Eigen::Vector3d& dir_in)
{
    for (unsigned int i = 0; i < n_frames_; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            dir_.col(i) = dir_in.normalized();
            return;
        }
    }
    throw_pretty("Could not find frame with name " << frame_name << ".");
}

Eigen::Vector3d EffAxisAlignment::get_axis(const std::string& frame_name)
{
    for (unsigned int i = 0; i < n_frames_; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            return axis_.col(i);
        }
    }
    throw_pretty("Axis for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::set_axis(const std::string& frame_name, const Eigen::Vector3d& axis_in)
{
    for (unsigned int i = 0; i < n_frames_; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            axis_.col(i) = axis_in.normalized();
            tf::vectorEigenToKDL(axis_.col(i), Frames[i + n_frames_].FrameAOffset.p);
            return;
        }
    }
    throw_pretty("Could not find frame with name " << frame_name << ".");
}
}
