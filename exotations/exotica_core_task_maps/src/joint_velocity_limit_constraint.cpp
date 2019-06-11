//
// Copyright (c) 2019, Christopher E. Mower
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

#include <exotica_core_task_maps/joint_velocity_limit_constraint.h>

REGISTER_TASKMAP_TYPE("JointVelocityLimitConstraint", exotica::JointVelocityLimitConstraint);

namespace exotica
{
void JointVelocityLimitConstraint::AssignScene(ScenePtr scene)
{
    scene_ = scene;

    // Get num dofs and double
    N_ = scene_->GetKinematicTree().GetNumControlledJoints();
    two_times_N_ = 2 * N_;

    // Get safe percentage and check
    double percent = static_cast<double>(parameters_.SafePercentage);
    if (percent > 1.0 || percent < 0.0) ThrowNamed("The safe percentage must be given such that it lies within the range [0, 1].");

    // Get current joint state
    if (parameters_.StartState.rows() != N_) ThrowNamed("Wrong size for start state.");
    current_joint_state_.resize(N_, 1);
    current_joint_state_ = parameters_.StartState;

    // Get joint velocity limits
    if (parameters_.MaximumJointVelocity.rows() == 1)
    {
        joint_velocity_limits_.setConstant(N_, 1, std::abs(static_cast<double>(parameters_.MaximumJointVelocity(0))));
    }
    else if (parameters_.MaximumJointVelocity.rows() == N_)
    {
        joint_velocity_limits_.resize(N_, 1);
        joint_velocity_limits_ = parameters_.MaximumJointVelocity.cwiseAbs();
    }
    else
    {
        ThrowNamed("Maximum joint velocity vector needs to be either of size 1 or N, but got " << parameters_.MaximumJointVelocity.rows());
    }
    joint_velocity_limits_ *= percent;

    // Compute 1/dt and init jacobian_
    one_divided_by_dt_ = 1.0 / parameters_.dt;

    jacobian_.resize(two_times_N_, N_);
    jacobian_.setZero();
    for (int i = 0; i < N_; ++i)
    {
        jacobian_(i, i) = one_divided_by_dt_;
        jacobian_(i + N_, i) = -one_divided_by_dt_;
    }
}

void JointVelocityLimitConstraint::SetPreviousJointState(Eigen::VectorXdRefConst joint_state)
{
    if (joint_state.rows() != N_) ThrowNamed("Wrong size for joint_state!");
    current_joint_state_ = joint_state;
}

void JointVelocityLimitConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    // Input check
    if (phi.rows() != two_times_N_) ThrowNamed("Wrong size of phi!");

    // Set phi
    Eigen::VectorXd x_dot = one_divided_by_dt_ * (x - current_joint_state_);
    for (int i = 0; i < N_; ++i)
    {
        phi(i) = x_dot(i) - joint_velocity_limits_(i);
        phi(i + N_) = -x_dot(i) - joint_velocity_limits_(i);
    }
}

void JointVelocityLimitConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    // Input check
    if (phi.rows() != two_times_N_) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != two_times_N_ || jacobian.cols() != N_) ThrowNamed("Wrong size of jacobian!");

    // Set phi and jacobian
    Update(x, phi);
    jacobian = jacobian_;
}

int JointVelocityLimitConstraint::TaskSpaceDim()
{
    return two_times_N_;
}

}  // namespace exotica
