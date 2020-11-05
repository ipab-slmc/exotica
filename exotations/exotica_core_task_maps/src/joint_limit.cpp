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

void JointLimit::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi)
{
    if (phi.rows() != N) ThrowNamed("Wrong size of phi!");

    const Eigen::MatrixXd& limits = scene_->GetKinematicTree().GetJointLimits();
    const Eigen::VectorXd& low_limits = limits.col(0);
    const Eigen::VectorXd& high_limits = limits.col(1);
    const Eigen::VectorXd tau = 0.5 * safe_percentage_ * (high_limits - low_limits);

    // apply lower bounds
    phi = (q.array() < (low_limits + tau).array()).select(q - low_limits - tau, phi);
    // apply higher bounds
    phi = (q.array() > (high_limits - tau).array()).select(q - high_limits + tau, phi);
}

void JointLimit::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (jacobian.rows() != N || jacobian.cols() != N) ThrowNamed("Wrong size of jacobian! " << N);
    Update(q, phi);

    // The jacobian is piece-wise: It's 0 when within limits, and 1 when outside.
    const Eigen::MatrixXd& limits = scene_->GetKinematicTree().GetJointLimits();
    const Eigen::VectorXd& low_limits = limits.col(0);
    const Eigen::VectorXd& high_limits = limits.col(1);
    const Eigen::VectorXd tau = 0.5 * safe_percentage_ * (high_limits - low_limits);
    for (int i = 0; i < N; i++)
    {
        if (q(i) >= low_limits(i) + tau(i) && q(i) <= high_limits(i) - tau(i))
        {
            jacobian(i, i) = 0;
        }
        else
        {
            jacobian(i, i) = 1;
        }
    }
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (hessian.size() != N) ThrowNamed("Wrong size of hessian! " << N);
    // Hessian is 0.
    Update(x, phi, jacobian);
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi)
{
    Update(x.head(scene_->get_num_positions()), phi);
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du)
{
    Update(x.head(scene_->get_num_positions()), phi, dphi_dx.topLeftCorner(N, N));
}

void JointLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du, HessianRef ddphi_ddx, HessianRef ddphi_ddu, HessianRef ddphi_dxdu)
{
    // Hessian is 0.
    Update(x.head(scene_->get_num_positions()), phi, dphi_dx.topLeftCorner(N, N));
}

}  // namespace exotica
