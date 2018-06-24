/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#include "task_map/Identity.h"

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);

namespace exotica
{
Identity::Identity()
{
}

void Identity::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != jointMap.size()) throw_named("Wrong size of phi!");
    for (int i = 0; i < jointMap.size(); i++)
    {
        phi(i) = x(jointMap[i]) - jointRef(i);
    }
}

void Identity::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != jointMap.size()) throw_named("Wrong size of phi!");
    if (J.rows() != jointMap.size() || J.cols() != N) throw_named("Wrong size of J! " << N);
    J.setZero();
    for (int i = 0; i < jointMap.size(); i++)
    {
        phi(i) = x(jointMap[i]) - jointRef(i);
        J(i, jointMap[i]) = 1.0;
    }
}

void Identity::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::VectorXdRef phidot, Eigen::MatrixXdRef J, Eigen::MatrixXdRef Jdot)
{
    if (phi.rows() != jointMap.size()) throw_named("Wrong size of phi!");
    if (J.rows() != jointMap.size() || J.cols() != N) throw_named("Wrong size of J! " << N);
    if (Jdot.rows() != jointMap.size() || Jdot.cols() != N) throw_named("Wrong size of J! " << N);
    J.setZero();
    Jdot.setZero();
    for (int i = 0; i < jointMap.size(); i++)
    {
        phi(i) = x(jointMap[i]) - jointRef(i);
        phidot(i) = x(jointMap[i] + N) - jointRef(i + jointMap.size());
        J(i, jointMap[i]) = 1.0;
        Jdot(i, jointMap[i]) = 1.0;
    }
}

void Identity::Instantiate(IdentityInitializer& init)
{
    init_ = init;
}

void Identity::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void Identity::Initialize()
{
    N = scene_->getSolver().getNumControlledJoints();
    if (init_.JointMap.rows() > 0)
    {
        jointMap.resize(init_.JointMap.rows());
        for(int i=0;i<init_.JointMap.rows();i++)
        {
            jointMap[i] = init_.JointMap(i);
        }
    }
    else
    {
        jointMap.resize(N);
        for (int i = 0; i < N; i++)
        {
            jointMap[i] = i;
        }
    }

    if (init_.JointRef.rows() > 0)
    {
        jointRef = init_.JointRef;
        if (jointRef.rows() != jointMap.size()) throw_named("Invalid joint reference size! Expecting " << jointMap.size());
    }
    else
    {
        jointRef = Eigen::VectorXd::Zero(jointMap.size());
    }
}

int Identity::taskSpaceDim()
{
    return jointMap.size();
}
}
