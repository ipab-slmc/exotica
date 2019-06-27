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

#include <exotica_core_task_maps/joint_limit.h>

REGISTER_TASKMAP_TYPE("JointLimit", exotica::JointLimit);

namespace exotica
{
void JointLimit::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void JointLimit::Initialize()
{
    safe_percentage_ = parameters_.SafePercentage;
    N = scene_->GetKinematicTree().GetNumControlledJoints();
}

int JointLimit::TaskSpaceDim()
{
    return N;
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != N) ThrowNamed("Wrong size of phi!");
    phi.setZero();

    const Eigen::MatrixXd limits = scene_->GetKinematicTree().GetJointLimits();
    const Eigen::VectorXd& low_limits = limits.col(0);
    const Eigen::VectorXd& high_limits = limits.col(1);
    const Eigen::VectorXd tau = 0.5 * safe_percentage_ * (high_limits - low_limits);

    // apply lower bounds
    phi = (x.array() < (low_limits + tau).array()).select(x - low_limits - tau, phi);
    // apply higher bounds
    phi = (x.array() > (high_limits - tau).array()).select(x - high_limits + tau, phi);
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    phi.setZero();
    Update(x, phi);

    if (jacobian.rows() != N || jacobian.cols() != N) ThrowNamed("Wrong size of jacobian! " << N);
    jacobian = Eigen::MatrixXd::Identity(N, N);
}
}  // namespace exotica
