//
// Copyright (c) 2018, Wolfgang Merkt
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

#include <exotica_core_task_maps/joint_velocity_limit.h>

REGISTER_TASKMAP_TYPE("JointVelocityLimit", exotica::JointVelocityLimit);

namespace exotica
{
JointVelocityLimit::JointVelocityLimit()
{
    kinematics.resize(2);  // Request kinematics for current x_t and x_{t-1}
}

JointVelocityLimit::~JointVelocityLimit() = default;

void JointVelocityLimit::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void JointVelocityLimit::Initialize()
{
    double percent = static_cast<double>(parameters_.SafePercentage);

    N = scene_->GetKinematicTree().GetNumControlledJoints();
    dt_ = std::abs(parameters_.dt);
    if (dt_ == 0.0)
        ThrowNamed("Timestep dt needs to be greater than 0");

    if (parameters_.MaximumJointVelocity.rows() == 1)
    {
        limits_.setOnes(N);
        limits_ *= std::abs(static_cast<double>(parameters_.MaximumJointVelocity(0)));
    }
    else if (parameters_.MaximumJointVelocity.rows() == N)
    {
        limits_ = parameters_.MaximumJointVelocity.cwiseAbs();
    }
    else
    {
        ThrowNamed("Maximum joint velocity vector needs to be either of size 1 or N, but got " << parameters_.MaximumJointVelocity.rows());
    }

    tau_ = percent * limits_;

    if (debug_) HIGHLIGHT_NAMED("JointVelocityLimit", "dt=" << dt_ << ", q̇_max=" << limits_.transpose() << ", τ=" << tau_.transpose());
}

int JointVelocityLimit::TaskSpaceDim()
{
    return N;
}

void JointVelocityLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (kinematics.size() != 2) ThrowNamed("Wrong size of kinematics - requires 2, but got " << kinematics.size());
    if (phi.rows() != N) ThrowNamed("Wrong size of phi!");
    if (!x.isApprox(kinematics[0].X)) ThrowNamed("The internal kinematics.X and passed state reference x do not match!\n x=" << std::setprecision(6) << x.transpose() << "\n X=" << kinematics[0].X.transpose() << "\n diff=" << (x - kinematics[0].X).transpose());

    phi.setZero();
    Eigen::VectorXd x_diff = (1 / dt_) * (kinematics[0].X - kinematics[1].X);
    for (int i = 0; i < N; ++i)
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

void JointVelocityLimit::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (jacobian.rows() != N || jacobian.cols() != N) ThrowNamed("Wrong size of jacobian! " << N);
    Update(x, phi);
    jacobian = (1 / dt_) * Eigen::MatrixXd::Identity(N, N);
    for (int i = 0; i < N; ++i)
        if (phi(i) == 0.0)
            jacobian(i, i) = 0.0;
}
}
