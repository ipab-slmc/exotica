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

#include "task_map/EffOrientation.h"

REGISTER_TASKMAP_TYPE("EffOrientation", exotica::EffOrientation);

namespace exotica
{
EffOrientation::EffOrientation() : rotationType(RotationType::RPY)
{
}

EffOrientation::~EffOrientation() = default;

void EffOrientation::Instantiate(EffOrientationInitializer& init)
{
    if (init.Type == "Quaternion")
    {
        rotationType = RotationType::QUATERNION;
    }
    else if (init.Type == "ZYX")
    {
        rotationType = RotationType::ZYX;
    }
    else if (init.Type == "ZYZ")
    {
        rotationType = RotationType::ZYZ;
    }
    else if (init.Type == "AngleAxis")
    {
        rotationType = RotationType::ANGLE_AXIS;
    }
    else if (init.Type == "Matrix")
    {
        rotationType = RotationType::MATRIX;
    }
    stride = getRotationTypeLength(rotationType);
}

std::vector<TaskVectorEntry> EffOrientation::getLieGroupIndices()
{
    std::vector<TaskVectorEntry> ret;
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        ret.push_back(TaskVectorEntry(Start + i * stride, rotationType));
    }
    return ret;
}

void EffOrientation::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != Kinematics[0].Phi.rows() * stride) throw_named("Wrong size of phi!");
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        phi.segment(i * stride, stride) = setRotation(Kinematics[0].Phi(i).M, rotationType);
    }
}

void EffOrientation::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != Kinematics[0].Phi.rows() * stride) throw_named("Wrong size of phi!");
    if (J.rows() != Kinematics[0].J.rows() * 3 || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        phi.segment(i * stride, stride) = setRotation(Kinematics[0].Phi(i).M, rotationType);
        J.middleRows(i * 3, 3) = Kinematics[0].J[i].data.bottomRows(3);
    }
}

int EffOrientation::taskSpaceDim()
{
    return Kinematics[0].Phi.rows() * stride;
}

int EffOrientation::taskSpaceJacobianDim()
{
    return Kinematics[0].Phi.rows() * 3;
}
}
