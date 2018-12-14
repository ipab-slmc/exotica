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

#include "task_map/LookAt.h"

REGISTER_TASKMAP_TYPE("LookAt", exotica::LookAt);

namespace exotica
{
LookAt::LookAt() = default;
LookAt::~LookAt() = default;

Eigen::Vector3d LookAt::getLookAtTarget()
{
    return Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(2).p.data);
}

void LookAt::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != 3) throw_named("Wrong size of phi!");

    // Get EffPoint and LookAtTarget
    Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(0).p.data);  // EffPoint | Eff frame
    Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(1).p.data);  // LookAtTarget | Eff frame

    // Compute orthogonal orthogonal projection a onto line e->c
    double c_squared_norm = c.squaredNorm();
    double p_dot_c = p.dot(c);
    double alpha = p_dot_c / c_squared_norm;
    Eigen::Vector3d a = alpha * c;

    // Set phi
    phi = a - p;
}

void LookAt::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != 3) throw_named("Wrong size of phi!");
    if (J.rows() != 3 || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    // Get EffPoint and LookAtTarget
    Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(0).p.data);  // EffPoint | Eff frame
    Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(1).p.data);  // LookAtTarget | Eff frame

    // Compute orthogonal orthogonal projection a onto line e->c
    double c_squared_norm = c.squaredNorm();
    double p_dot_c = p.dot(c);
    double alpha = p_dot_c / c_squared_norm;
    Eigen::Vector3d a = alpha * c;

    // Set phi
    phi = a - p;

    // Compute J
    for (int j = 0; j < J.cols(); j++)
    {
        Eigen::Vector3d pd = Kinematics[0].J[1].data.topRows<3>().col(j);
        double alphad = c.dot(pd) / c_squared_norm;
        J.col(j) = alphad * c - pd;
    }
}

void LookAt::Instantiate(LookAtInitializer &init)
{
    if (Frames.size() < 3) throw_named("Three frames are required!");
}

int LookAt::taskSpaceDim()
{
    return 3;
}
}  // namespace exotica
