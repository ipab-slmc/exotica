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

#include "task_map/JointVelocityLimit.h"

REGISTER_TASKMAP_TYPE("JointVelocityLimit", exotica::JointVelocityLimit);

namespace exotica
{
JointVelocityLimit::JointVelocityLimit()
{
    Kinematics.resize(2);  // Request kinematics for current x_t and x_{t-1}
}

JointVelocityLimit::~JointVelocityLimit() = default;

void JointVelocityLimit::Instantiate(JointVelocityLimitInitializer& init)
{
    init_ = init;
}

void JointVelocityLimit::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void JointVelocityLimit::Initialize()
{
    double percent = static_cast<double>(init_.SafePercentage);

    N = scene_->getSolver().getNumControlledJoints();
    dt_ = std::abs(init_.dt);
    if (dt_ == 0.0)
        throw_named("Timestep dt needs to be greater than 0");

    if (init_.MaximumJointVelocity.rows() == 1)
    {
        limits_.setOnes(N);
        limits_ *= std::abs(static_cast<double>(init_.MaximumJointVelocity(0)));
    }
    else if (init_.MaximumJointVelocity.rows() == N)
    {
        limits_ = init_.MaximumJointVelocity.cwiseAbs();
    }
    else
    {
        throw_named("Maximum joint velocity vector needs to be either of size 1 or N, but got " << init_.MaximumJointVelocity.rows());
    }

    tau_ = percent * limits_;

    if (debug_) HIGHLIGHT_NAMED("JointVelocityLimit", "dt=" << dt_ << ", q̇_max=" << limits_.transpose() << ", τ=" << tau_.transpose());
}

int JointVelocityLimit::taskSpaceDim()
{
    return N;
}

void JointVelocityLimit::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (Kinematics.size() != 2) throw_named("Wrong size of Kinematics - requires 2, but got " << Kinematics.size());
    if (phi.rows() != N) throw_named("Wrong size of phi!");
    if (!x.isApprox(Kinematics[0].X)) throw_named("The internal Kinematics.X and passed state reference x do not match!");

    phi.setZero();
    Eigen::VectorXd x_diff = (1 / dt_) * (Kinematics[0].X - Kinematics[1].X);
    for (int i = 0; i < N; i++)
    {
        if (x_diff(i) < -limits_(i) + tau_(i))
        {
            phi(i) = x_diff(i) + limits_(i) - tau_(i);
            if (debug_) HIGHLIGHT_NAMED("JointVelocityLimit", "Lower limit exceeded (joint=" << i << "): " << x_diff(i) << " < (-" << limits_(i) << "+" << tau_(i) << ")");
        }
        if (x_diff(i) > limits_(i) - tau_(i))
        {
            phi(i) = x_diff(i) - limits_(i) + tau_(i);
            if (debug_) HIGHLIGHT_NAMED("JointVelocityLimit", "Upper limit exceeded (joint=" << i << "): " << x_diff(i) << " > (" << limits_(i) << "-" << tau_(i) << ")");
        }
    }
}

void JointVelocityLimit::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (J.rows() != N || J.cols() != N) throw_named("Wrong size of J! " << N);
    update(x, phi);
    J = (1 / dt_) * Eigen::MatrixXd::Identity(N, N);
    for (int i = 0; i < N; i++)
        if (phi(i) == 0.0)
            J(i, i) = 0.0;
}
}
