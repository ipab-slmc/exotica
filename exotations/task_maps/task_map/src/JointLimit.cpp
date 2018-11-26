/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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

#include "task_map/JointLimit.h"

REGISTER_TASKMAP_TYPE("JointLimit", exotica::JointLimit);

namespace exotica
{
JointLimit::JointLimit() = default;

void JointLimit::Instantiate(JointLimitInitializer& init)
{
    init_ = init;
}

void JointLimit::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void JointLimit::Initialize()
{
    double percent = init_.SafePercentage;

    N = scene_->getSolver().getNumControlledJoints();

    // TODO: Strictly speaking this is incorrect as joint limits can be changed during runtime.
    Eigen::MatrixXd limits = scene_->getSolver().getJointLimits();

    low_limits_.resize(N);
    high_limits_.resize(N);
    tau_.resize(N);
    for (int i = 0; i < N; i++)
    {
        low_limits_(i) = limits(i, 0);
        high_limits_(i) = limits(i, 1);
        tau_(i) = percent * (high_limits_(i) - low_limits_(i)) * 0.5;
    }
}

int JointLimit::taskSpaceDim()
{
    return N;
}

void JointLimit::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != N) throw_named("Wrong size of phi!");
    phi.setZero();
    for (int i = 0; i < N; i++)
    {
        if (x(i) < low_limits_(i) + tau_(i))
        {
            phi(i) = x(i) - low_limits_(i) - tau_(i);
        }
        if (x(i) > high_limits_(i) - tau_(i))
        {
            phi(i) = x(i) - high_limits_(i) + tau_(i);
        }
    }
}

void JointLimit::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    phi.setZero();
    update(x, phi);

    if (J.rows() != N || J.cols() != N) throw_named("Wrong size of J! " << N);
    J = Eigen::MatrixXd::Identity(N, N);
}
}
