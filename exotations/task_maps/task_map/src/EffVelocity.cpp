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

#include "task_map/EffVelocity.h"

REGISTER_TASKMAP_TYPE("EffVelocity", exotica::EffVelocity);

namespace exotica
{
EffVelocity::EffVelocity()
{
    Kinematics.resize(2);
}

void EffVelocity::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (Kinematics.size() != 2) throw_named("Wrong size of Kinematics - requires 2, but got " << Kinematics.size());
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");

    Eigen::Vector3d p_t, p_t_prev;
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, p_t);
        tf::vectorKDLToEigen(Kinematics[1].Phi(i).p, p_t_prev);
        phi(i) = (p_t_prev - p_t).norm();
    }
}

void EffVelocity::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (Kinematics.size() != 2) throw_named("Wrong size of Kinematics - requires 2, but got " << Kinematics.size());
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");
    if (J.rows() != Kinematics[0].J.rows() || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    J.setZero();
    Eigen::Vector3d p_t, p_t_prev;
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        tf::vectorKDLToEigen(Kinematics[0].Phi(i).p, p_t);
        tf::vectorKDLToEigen(Kinematics[1].Phi(i).p, p_t_prev);
        phi(i) = (p_t - p_t_prev).norm();

        if (phi(i) != 0.0)
        {
            J.row(i) = ((p_t(0) - p_t_prev(0)) * Kinematics[0].J[i].data.row(0) +
                        (p_t(1) - p_t_prev(1)) * Kinematics[0].J[i].data.row(1) +
                        (p_t(2) - p_t_prev(2)) * Kinematics[0].J[i].data.row(2)) /
                       phi(i);
        }
    }
}

void EffVelocity::Instantiate(EffVelocityInitializer& init)
{
}

int EffVelocity::taskSpaceDim()
{
    return Kinematics[0].Phi.rows();
}
}
