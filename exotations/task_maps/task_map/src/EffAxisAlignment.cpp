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

#include "task_map/EffAxisAlignment.h"

REGISTER_TASKMAP_TYPE("EffAxisAlignment", exotica::EffAxisAlignment);

namespace exotica
{
EffAxisAlignment::EffAxisAlignment() = default;
EffAxisAlignment::~EffAxisAlignment() = default;

void EffAxisAlignment::Instantiate(EffAxisAlignmentInitializer& init)
{
    init_ = init;
}

void EffAxisAlignment::Initialize()
{
    N = scene_->getKinematicTree().getNumControlledJoints();

    NumberOfFrames = init_.EndEffector.size();
    if (debug_) HIGHLIGHT_NAMED("EffAxisAlignment", "Number of EndEffectors: " << NumberOfFrames);
    axis_.resize(3, NumberOfFrames);
    dir_.resize(3, NumberOfFrames);

    Frames.resize(2 * NumberOfFrames);
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        FrameWithAxisAndDirectionInitializer frame(init_.EndEffector[i]);
        axis_.col(i) = frame.Axis.normalized();
        dir_.col(i) = frame.Direction.normalized();

        Frames[i + NumberOfFrames] = Frames[i];
        tf::vectorEigenToKDL(axis_.col(i), Frames[i + NumberOfFrames].FrameAOffset.p);
    }

    if (debug_)
    {
        for (unsigned int i = 0; i < NumberOfFrames; i++)
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
    Initialize();
}

void EffAxisAlignment::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != NumberOfFrames) throw_named("Wrong size of phi!");

    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, linkPositionInBase_);
        tf::vectorKDLToEigen(Kinematics[0].Phi(i + NumberOfFrames).p, linkAxisPositionInBase_);

        Eigen::Vector3d axisInBase = linkAxisPositionInBase_ - linkPositionInBase_;
        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
    }
}

void EffAxisAlignment::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != NumberOfFrames) throw_named("Wrong size of phi!");
    if (J.rows() != NumberOfFrames || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, linkPositionInBase_);
        tf::vectorKDLToEigen(Kinematics[0].Phi(i + NumberOfFrames).p, linkAxisPositionInBase_);

        Eigen::Vector3d axisInBase = linkAxisPositionInBase_ - linkPositionInBase_;
        Eigen::MatrixXd axisInBaseJacobian = Kinematics[0].J[i + NumberOfFrames].data.block(0, 0, 3, N) - Kinematics[0].J[i].data.block(0, 0, 3, N);

        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
        J.row(i) = dir_.col(i).transpose() * axisInBaseJacobian;
    }
}

int EffAxisAlignment::taskSpaceDim()
{
    return NumberOfFrames;
}

Eigen::Vector3d EffAxisAlignment::getDirection(const std::string& frame_name)
{
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            return dir_.col(i);
        }
    }
    throw_pretty("Direction for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::setDirection(const std::string& frame_name, const Eigen::Vector3d& dir_in)
{
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            dir_.col(i) = dir_in.normalized();
            return;
        }
    }
    throw_pretty("Could not find frame with name " << frame_name << ".");
}

Eigen::Vector3d EffAxisAlignment::getAxis(const std::string& frame_name)
{
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            return axis_.col(i);
        }
    }
    throw_pretty("Axis for frame with name " << frame_name << " could not be found.");
}

void EffAxisAlignment::setAxis(const std::string& frame_name, const Eigen::Vector3d& axis_in)
{
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        if (Frames[i].FrameALinkName == frame_name)
        {
            axis_.col(i) = axis_in.normalized();
            tf::vectorEigenToKDL(axis_.col(i), Frames[i + NumberOfFrames].FrameAOffset.p);
            return;
        }
    }
    throw_pretty("Could not find frame with name " << frame_name << ".");
}
}
