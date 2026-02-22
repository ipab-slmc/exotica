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

#include <exotica_core_task_maps/eff_orientation.h>

REGISTER_TASKMAP_TYPE("EffOrientation", exotica::EffOrientation);

namespace exotica
{
void EffOrientation::Instantiate(const EffOrientationInitializer& init)
{
    rotation_type_ = GetRotationTypeFromString(init.Type);
    stride_ = GetRotationTypeLength(rotation_type_);
}

std::vector<TaskVectorEntry> EffOrientation::GetLieGroupIndices()
{
    std::vector<TaskVectorEntry> ret;
    ret.reserve(kinematics[0].Phi.rows());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        ret.emplace_back(TaskVectorEntry(start + i * stride_, rotation_type_));
    }
    return ret;
}

void EffOrientation::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows() * stride_) ThrowNamed("Wrong size of Phi! Expected " << kinematics[0].Phi.rows() * stride_ << ", but received " << phi.rows());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi.segment(i * stride_, stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
    }
}

void EffOrientation::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * stride_) ThrowNamed("Wrong size of Phi! Expected " << kinematics[0].Phi.rows() * stride_ << ", but received " << phi.rows());
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 3 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi.segment(i * stride_, stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
        jacobian.middleRows<3>(i * 3) = kinematics[0].jacobian[i].data.bottomRows<3>();
    }
}

void EffOrientation::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != kinematics[0].Phi.rows() * stride_) ThrowNamed("Wrong size of Phi! Expected " << kinematics[0].Phi.rows() * stride_ << ", but received " << phi.rows());
    if (jacobian.rows() != kinematics[0].jacobian.rows() * 3 || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi.segment(i * stride_, stride_) = SetRotation(kinematics[0].Phi(i).M, rotation_type_);
        jacobian.middleRows<3>(i * 3) = kinematics[0].jacobian[i].data.bottomRows<3>();

        for (int j = 0; j < 3; ++j)
        {
            hessian(i * 3 + j).block(0, 0, jacobian.cols(), jacobian.cols()) = kinematics[0].hessian[i](j + 3);
        }
    }
}

int EffOrientation::TaskSpaceDim()
{
    return kinematics[0].Phi.rows() * stride_;
}

int EffOrientation::TaskSpaceJacobianDim()
{
    return kinematics[0].Phi.rows() * 3;
}

const RotationType& EffOrientation::get_rotation_type() const
{
    return rotation_type_;
}

}  // namespace exotica
