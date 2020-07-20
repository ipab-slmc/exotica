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

#include <exotica_core_task_maps/joint_pose.h>

REGISTER_TASKMAP_TYPE("JointPose", exotica::JointPose);

namespace exotica
{
void JointPose::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi)
{
    if (phi.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Wrong size of Phi!");
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(i) = q(joint_map_[i]) - joint_ref_(i);
    }
}

void JointPose::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != static_cast<int>(joint_map_.size()) || jacobian.cols() != num_controlled_joints_) ThrowNamed("Wrong size of jacobian! " << num_controlled_joints_);
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(i) = q(joint_map_[i]) - joint_ref_(i);
        jacobian(i, joint_map_[i]) = 1.0;
    }
}

void JointPose::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    // Hessian is 0
    Update(q, phi, jacobian);
}

void JointPose::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void JointPose::Initialize()
{
    num_controlled_joints_ = scene_->GetKinematicTree().GetNumControlledJoints();
    if (parameters_.JointMap.rows() > 0)
    {
        joint_map_.resize(parameters_.JointMap.rows());
        for (int i = 0; i < parameters_.JointMap.rows(); ++i)
        {
            joint_map_[i] = parameters_.JointMap(i);
        }
    }
    else
    {
        joint_map_.resize(num_controlled_joints_);
        for (int i = 0; i < num_controlled_joints_; ++i)
        {
            joint_map_[i] = i;
        }
    }

    if (parameters_.JointRef.rows() > 0)
    {
        joint_ref_ = parameters_.JointRef;
        if (joint_ref_.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Invalid joint reference size! Expecting " << joint_map_.size() << " but received " << joint_ref_.rows());
    }
    else
    {
        joint_ref_ = Eigen::VectorXd::Zero(joint_map_.size());
    }
}

int JointPose::TaskSpaceDim()
{
    return joint_map_.size();
}

const std::vector<int>& JointPose::get_joint_map() const
{
    return joint_map_;
}

const Eigen::VectorXd& JointPose::get_joint_ref() const
{
    return joint_ref_;
}

void JointPose::set_joint_ref(Eigen::VectorXdRefConst ref)
{
    if (ref.size() == joint_ref_.size())
        joint_ref_ = ref;
    else
        ThrowPretty("Wrong size - expected " << joint_ref_.size() << ", but received " << ref.size());
}

}  // namespace exotica
