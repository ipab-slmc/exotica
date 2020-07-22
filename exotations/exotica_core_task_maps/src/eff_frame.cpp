//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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
void EffFrame::Instantiate(const EffFrameInitializer& init)
{
    rotation_type_ = GetRotationTypeFromString(init.Type);
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
        phi.segment<3>(i * big_stride_) = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi.segment(i * big_stride_ + 3, small_stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
    }
}

void EffFrame::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * big_stride_) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 6 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi.segment<3>(i * big_stride_) = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi.segment(i * big_stride_ + 3, small_stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
        jacobian.middleRows<6>(i * 6) = kinematics[0].jacobian[i].data;
    }
}

void EffFrame::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * big_stride_) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 6 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi.segment<3>(i * big_stride_) = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi.segment(i * big_stride_ + 3, small_stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
        jacobian.middleRows<6>(i * 6) = kinematics[0].jacobian[i].data;

        for (int j = 0; j < 6; ++j)
        {
            hessian(i * 6 + j).block(0, 0, jacobian.cols(), jacobian.cols()) = kinematics[0].hessian[i](j);
        }
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

const RotationType& EffFrame::get_rotation_type() const
{
    return rotation_type_;
}

}  // namespace exotica
