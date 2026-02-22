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

#include <exotica_core_task_maps/joint_acceleration_backward_difference.h>

REGISTER_TASKMAP_TYPE("JointAccelerationBackwardDifference", exotica::JointAccelerationBackwardDifference);

namespace exotica
{
void JointAccelerationBackwardDifference::AssignScene(ScenePtr scene)
{
    scene_ = scene;

    // Get ndof
    N_ = scene_->GetKinematicTree().GetNumControlledJoints();

    // Set binomial coefficient parameters
    backward_difference_params_ << -2, 1;

    // Init each col of q_ with start state
    q_.resize(N_, 2);
    if (parameters_.StartState.rows() == 0)
    {
        q_.setZero(N_, 2);
    }
    else if (parameters_.StartState.rows() == N_)
    {
        for (int i = 0; i < 2; ++i)
            q_.col(i) = parameters_.StartState;
    }
    else
    {
        ThrowPretty("Wrong size for StartState!");
    }

    // Init qbd_
    qbd_ = q_ * backward_difference_params_;

    // Init identity matrix
    I_ = Eigen::MatrixXd::Identity(N_, N_);
}

void JointAccelerationBackwardDifference::SetPreviousJointState(Eigen::VectorXdRefConst joint_state)
{
    // Input check
    if (joint_state.rows() != N_) ThrowNamed("Wrong size for joint_state!");

    // Push back previous joint states
    q_.col(1) = q_.col(0);
    q_.col(0) = joint_state;

    // Compute new qbd_
    qbd_ = q_ * backward_difference_params_;
}

void JointAccelerationBackwardDifference::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    // Input check
    if (phi.rows() != N_) ThrowNamed("Wrong size of phi!");

    // Estimate second time derivative
    phi = x + qbd_;
}

void JointAccelerationBackwardDifference::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    // Input check
    if (jacobian.rows() != N_ || jacobian.cols() != N_) ThrowNamed("Wrong size of jacobian! " << N_);

    // Estimate second time derivative and set Jacobian to identity matrix
    Update(x, phi);
    jacobian = I_;
}

int JointAccelerationBackwardDifference::TaskSpaceDim()
{
    return N_;
}
}  // namespace exotica
