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
EffAxisAlignment::EffAxisAlignment()
{
}

void EffAxisAlignment::Instantiate(EffAxisAlignmentInitializer& init)
{
    init_ = init;
}

void EffAxisAlignment::Initialize()
{
    N = scene_->getSolver().getNumControlledJoints();
    NumberOfFrames = Frames.size();
    axis_.resize(3, NumberOfFrames);
    dir_.resize(3, NumberOfFrames);

    if (init_.Axis.size() > 0)
    {
        if (init_.Axis.size() == 3)
        {
            double axis_length = init_.Axis.norm();
            for (unsigned int i = 0; i < NumberOfFrames; i++)
            {
                axis_.col(i) = init_.Axis / axis_length;
            }
        }
        else if (init_.Axis.size() == 3 * NumberOfFrames)
        {
            for (unsigned int i = 0; i < NumberOfFrames; i++)
            {
                double axis_length = init_.Axis.segment(3 * i, 3).norm();
                axis_.col(i) = init_.Axis.segment(3 * i, 3) / axis_length;
            }
        }
        else
        {
            throw_pretty("Size of Axis needs to be three or the number of frames times three, got " << init_.Axis.size());
        }
    }
    else
    {
        for (unsigned int i = 0; i < NumberOfFrames; i++) axis_.col(i) = Eigen::Vector3d(0, 0, 1);
    }

    if (init_.Direction.size() > 0)
    {
        if (init_.Direction.size() == 3)
        {
            double dir_length = init_.Direction.norm();
            for (unsigned int i = 0; i < NumberOfFrames; i++)
            {
                dir_.col(i) = init_.Direction / dir_length;
            }
        }
        else if (init_.Direction.size() == 3 * NumberOfFrames)
        {
            for (unsigned int i = 0; i < NumberOfFrames; i++)
            {
                double dir_length = init_.Direction.segment(3 * i, 3).norm();
                dir_.col(i) = init_.Direction.segment(3 * i, 3) / dir_length;
            }
        }
        else
        {
            throw_pretty("Size of Direction needs to be three or the number of frames times three, got " << init_.Direction.size());
        }
    }
    else
    {
        for (unsigned int i = 0; i < NumberOfFrames; i++) dir_.col(i) = Eigen::Vector3d(0, 0, 1);
    }

    Frames.resize(2 * NumberOfFrames);
    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        Frames[i + NumberOfFrames].FrameALinkName = Frames[i].FrameALinkName;
        Frames[i + NumberOfFrames].FrameAOffset.p = KDL::Vector(axis_.col(i)(0), axis_.col(i)(1), axis_.col(i)(2));
    }

    for (unsigned int i = 0; i < Frames.size(); i++)
    {
        if (debug_)
            HIGHLIGHT_NAMED("EffAxisAlignment",
                            "FrameALinkName: "
                                << Frames[i].FrameALinkName
                                << ", offset: ("
                                << Frames[i].FrameAOffset.p.x() << ", "
                                << Frames[i].FrameAOffset.p.y() << ", "
                                << Frames[i].FrameAOffset.p.z() << ")");
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
        tf::vectorKDLToEigen(Kinematics.Phi(i).p, linkPositionInBase_);
        tf::vectorKDLToEigen(Kinematics.Phi(i + NumberOfFrames).p, linkAxisPositionInBase_);

        Eigen::Vector3d axisInBase = linkAxisPositionInBase_ - linkPositionInBase_;
        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
    }
}

void EffAxisAlignment::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != NumberOfFrames) throw_named("Wrong size of phi!");
    if (J.rows() != NumberOfFrames || J.cols() != Kinematics.J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics.J(0).data.cols());

    for (unsigned int i = 0; i < NumberOfFrames; i++)
    {
        tf::vectorKDLToEigen(Kinematics.Phi(i).p, linkPositionInBase_);
        tf::vectorKDLToEigen(Kinematics.Phi(i + NumberOfFrames).p, linkAxisPositionInBase_);

        Eigen::Vector3d axisInBase = linkAxisPositionInBase_ - linkPositionInBase_;
        Eigen::MatrixXd axisInBaseJacobian = Kinematics.J[i + NumberOfFrames].data.block(0, 0, 3, N) - Kinematics.J[i].data.block(0, 0, 3, N);

        phi(i) = axisInBase.dot(dir_.col(i)) - 1.0;
        J.row(i) = dir_.col(i).transpose() * axisInBaseJacobian;
    }
}

int EffAxisAlignment::taskSpaceDim()
{
    return NumberOfFrames;
}
}
