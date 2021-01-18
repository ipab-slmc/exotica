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

#include <eigen_conversions/eigen_kdl.h>

#include <exotica_core_task_maps/eff_velocity.h>

REGISTER_TASKMAP_TYPE("EffVelocity", exotica::EffVelocity);

namespace exotica
{
EffVelocity::EffVelocity()
{
    kinematics.resize(2);
}

EffVelocity::~EffVelocity() = default;

void EffVelocity::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (kinematics.size() != 2) ThrowNamed("Wrong size of kinematics - requires 2, but got " << kinematics.size());
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");

    Eigen::Vector3d p_t, p_t_prev;
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        tf::vectorKDLToEigen(kinematics[0].Phi(i).p, p_t);
        tf::vectorKDLToEigen(kinematics[1].Phi(i).p, p_t_prev);
        phi(i) = (p_t_prev - p_t).norm();
    }
}

void EffVelocity::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (kinematics.size() != 2) ThrowNamed("Wrong size of kinematics - requires 2, but got " << kinematics.size());
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    jacobian.setZero();
    Eigen::Vector3d p_t, p_t_prev;
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        tf::vectorKDLToEigen(kinematics[0].Phi(i).p, p_t);
        tf::vectorKDLToEigen(kinematics[1].Phi(i).p, p_t_prev);
        phi(i) = (p_t - p_t_prev).norm();

        if (phi(i) != 0.0)
        {
            jacobian.row(i) = ((p_t(0) - p_t_prev(0)) * kinematics[0].jacobian[i].data.row(0) +
                               (p_t(1) - p_t_prev(1)) * kinematics[0].jacobian[i].data.row(1) +
                               (p_t(2) - p_t_prev(2)) * kinematics[0].jacobian[i].data.row(2)) /
                              phi(i);
        }
    }
}

int EffVelocity::TaskSpaceDim()
{
    return kinematics[0].Phi.rows();
}
}  // namespace exotica
