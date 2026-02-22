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

#include <exotica_core_task_maps/joint_torque_minimization_proxy.h>

REGISTER_TASKMAP_TYPE("JointTorqueMinimizationProxy", exotica::JointTorqueMinimizationProxy);

namespace exotica
{
void JointTorqueMinimizationProxy::Instantiate(const JointTorqueMinimizationProxyInitializer& init)
{
    parameters_ = init;
    if (init.h.size() != 6)
    {
        ThrowPretty("Size of selection vector h needs to be 6, got " << init.h.size());
    }
    h_ = init.h;
}

Eigen::Matrix<double, 6, 1> JointTorqueMinimizationProxy::get_h() const
{
    return h_;
}

void JointTorqueMinimizationProxy::set_h(const Eigen::Matrix<double, 6, 1>& h_in)
{
    if (h_in.size() != 6)
        ThrowPretty("Wrong size!");
    h_ = h_in;
}

void JointTorqueMinimizationProxy::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != static_cast<int>(frames_.size())) ThrowNamed("Wrong size of Phi!");
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = h_.transpose() * kinematics[0].jacobian[i].data * kinematics[0].jacobian[i].data.transpose() * h_;
    }
}

// void JointTorqueMinimizationProxy::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
// {
//     if (phi.rows() != frames_.size()) ThrowNamed("Wrong size of Phi!");
//     for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
//     {
//         phi(i) = h_.transpose() * kinematics[0].jacobian[i].data * kinematics[0].jacobian[i].data.transpose() * h_;
//         Eigen::MatrixXd Jdot = scene_->GetKinematicTree().Jdot(kinematics[0].jacobian[i]);
//         HIGHLIGHT("Jdot size:" << Jdot.rows() << "x" << Jdot.cols())
//         HIGHLIGHT("Jacobian size:" << kinematics[0].jacobian[i].data.rows() << "x" << kinematics[0].jacobian[i].data.cols())
//         HIGHLIGHT("h_ size:" << h_.rows() << "x" << h_.cols())
//         HIGHLIGHT("jacobian-i size:" << jacobian.row(i).rows() << "x" << jacobian.row(i).cols())
//         jacobian.row(i) = 2 * (h_.transpose() * Jdot).transpose() * kinematics[0].jacobian[i].data;
//     }
// }

int JointTorqueMinimizationProxy::TaskSpaceDim()
{
    return frames_.size();
}
}  // namespace exotica
