//
// Copyright (c) 2019, Wolfgang Merkt
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

#include <exotica_core_task_maps/continuous_joint_pose.h>

REGISTER_TASKMAP_TYPE("ContinuousJointPose", exotica::ContinuousJointPose);

namespace exotica
{
void ContinuousJointPose::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != N_) ThrowNamed("Wrong size of Phi!");
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(2 * i + 0) = std::cos(x(joint_map_[i]));
        phi(2 * i + 1) = std::sin(x(joint_map_[i]));
    }
}

void ContinuousJointPose::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != N_) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != N_) ThrowNamed("Wrong size of jacobian! " << N_);
    jacobian.setZero();
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(2 * i + 0) = std::cos(x(joint_map_[i]));
        phi(2 * i + 1) = std::sin(x(joint_map_[i]));

        // Jacobian
        jacobian(2 * i + 0, joint_map_[i]) = -std::sin(x(joint_map_[i]));
        jacobian(2 * i + 1, joint_map_[i]) = std::cos(x(joint_map_[i]));
    }
}

void ContinuousJointPose::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != N_) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != N_) ThrowNamed("Wrong size of jacobian! " << N_);
    if (hessian.size() != N_) ThrowNamed("Wrong size of Hessian!" << N_ << " vs " << hessian.size());

    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(2 * i + 0) = std::cos(x(joint_map_[i]));
        phi(2 * i + 1) = std::sin(x(joint_map_[i]));

        // Jacobian
        jacobian(2 * i + 0, joint_map_[i]) = -std::sin(x(joint_map_[i]));
        jacobian(2 * i + 1, joint_map_[i]) = std::cos(x(joint_map_[i]));

        // Hessian
        hessian(2 * i + 0)(joint_map_[i], joint_map_[i]) = -std::cos(x(joint_map_[i]));
        hessian(2 * i + 1)(joint_map_[i], joint_map_[i]) = -std::sin(x(joint_map_[i]));
    }
}

void ContinuousJointPose::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void ContinuousJointPose::Initialize()
{
    const int num_controlled_joints = scene_->GetKinematicTree().GetNumControlledJoints();
    if (parameters_.JointMap.rows() > 0)
    {
        if (parameters_.JointMap.rows() > num_controlled_joints)
            ThrowPretty("Number of mapped joints greater than controlled joints!");

        joint_map_.resize(parameters_.JointMap.rows());
        for (int i = 0; i < parameters_.JointMap.rows(); ++i)
        {
            joint_map_[i] = parameters_.JointMap(i);
        }
    }
    else
    {
        joint_map_.resize(num_controlled_joints);
        for (int i = 0; i < num_controlled_joints; ++i)
        {
            joint_map_[i] = i;
        }
    }
    N_ = 2 * joint_map_.size();
}

int ContinuousJointPose::TaskSpaceDim()
{
    return N_;
}
}  // namespace exotica
