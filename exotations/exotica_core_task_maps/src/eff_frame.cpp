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

#include <exotica_core_task_maps/eff_frame.h>

REGISTER_TASKMAP_TYPE("EffFrame", exotica::EffFrame);

namespace exotica
{
EffFrame::EffFrame() = default;
EffFrame::~EffFrame() = default;

void EffFrame::Instantiate(const EffFrameInitializer &init)
{
    if (init.Type == "Quaternion")
    {
        rotation_type_ = RotationType::QUATERNION;
    }
    else if (init.Type == "ZYX")
    {
        rotation_type_ = RotationType::ZYX;
    }
    else if (init.Type == "ZYZ")
    {
        rotation_type_ = RotationType::ZYZ;
    }
    else if (init.Type == "AngleAxis")
    {
        rotation_type_ = RotationType::ANGLE_AXIS;
    }
    else if (init.Type == "Matrix")
    {
        rotation_type_ = RotationType::MATRIX;
    }
    small_stride_ = GetRotationTypeLength(rotation_type_);
    big_stride_ = small_stride_ + 3;
}

std::vector<TaskVectorEntry> EffFrame::GetLieGroupIndices()
{
    std::vector<TaskVectorEntry> ret;
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        ret.push_back(TaskVectorEntry(start + i * big_stride_ + 3, rotation_type_));
    }
    return ret;
}

void EffFrame::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows() * big_stride_) ThrowNamed("Wrong size of Phi!");
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i * big_stride_) = kinematics[0].Phi(i).p[0];
        phi(i * big_stride_ + 1) = kinematics[0].Phi(i).p[1];
        phi(i * big_stride_ + 2) = kinematics[0].Phi(i).p[2];
        phi.segment(i * big_stride_ + 3, small_stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
    }
}

void EffFrame::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * big_stride_) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 6 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i * big_stride_) = kinematics[0].Phi(i).p[0];
        phi(i * big_stride_ + 1) = kinematics[0].Phi(i).p[1];
        phi(i * big_stride_ + 2) = kinematics[0].Phi(i).p[2];
        phi.segment(i * big_stride_ + 3, small_stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
        jacobian.middleRows(i * 6, 6) = kinematics[0].jacobian[i].data;
    }
}

int EffFrame::TaskSpaceDim()
{
    return kinematics[0].Phi.rows() * big_stride_;
}

int EffFrame::TaskSpaceJacobianDim()
{
    return kinematics[0].Phi.rows() * 6;
}
}
