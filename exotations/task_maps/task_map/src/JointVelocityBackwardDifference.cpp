/*
 *      Author: Christopher E. Mower
 *
 * Copyright (c) 2018, University of Edinburgh
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

#include "task_map/JointVelocityBackwardDifference.h"

REGISTER_TASKMAP_TYPE("JointVelocityBackwardDifference", exotica::JointVelocityBackwardDifference);

namespace exotica
{
JointVelocityBackwardDifference::JointVelocityBackwardDifference() = default;
JointVelocityBackwardDifference::~JointVelocityBackwardDifference() = default;

void JointVelocityBackwardDifference::assignScene(Scene_ptr scene)
{
    scene_ = scene;

    // Get ndof
    N_ = scene_->getSolver().getNumControlledJoints();

    // Set binomial coefficient parameters
    backward_difference_params_ = -1.0;

    // Frequency
    if (init_.dt <= 0) throw_pretty("dt cannot be smaller than or equal to 0.");
    dt_inv_ = 1 / init_.dt;

    // Init each col of q_ with start state
    q_.resize(N_, 1);
    if (init_.StartState.rows() == 0)
        q_.setZero(N_);
    else if (init_.StartState.rows() == N_)
        q_ = init_.StartState;
    else
        throw_pretty("Wrong size for StartState!");

    // Init qbd_
    qbd_ = q_ * backward_difference_params_;

    // Init identity matrix
    I_ = Eigen::MatrixXd::Identity(N_, N_);
}

void JointVelocityBackwardDifference::Instantiate(JointVelocityBackwardDifferenceInitializer& init)
{
    init_ = init;
}

void JointVelocityBackwardDifference::SetPreviousJointState(Eigen::VectorXdRefConst joint_state)
{
    // Input check
    if (joint_state.rows() != N_) throw_named("Wrong size for joint_state!");

    // Push back previous joint states
    q_ = joint_state;

    // Compute new qbd_
    qbd_ = q_ * backward_difference_params_;
}

void JointVelocityBackwardDifference::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    // Input check
    if (phi.rows() != N_) throw_named("Wrong size of phi!");

    // Estimate third time derivative
    phi = dt_inv_ * (x + qbd_);
}

void JointVelocityBackwardDifference::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    // Input check
    if (phi.rows() != N_) throw_named("Wrong size of phi!");
    if (J.rows() != N_ || J.cols() != N_) throw_named("Wrong size of J! " << N_);

    // Estimate third time derivative and set Jacobian to identity matrix
    phi = dt_inv_ * (x + qbd_);
    J = dt_inv_ * I_;
}

int JointVelocityBackwardDifference::taskSpaceDim()
{
    return N_;
}
}
