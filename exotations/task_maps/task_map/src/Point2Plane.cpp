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

#include "task_map/Point2Plane.h"

REGISTER_TASKMAP_TYPE("Point2Plane", exotica::Point2Plane);

namespace exotica
{
Point2Plane::Point2Plane()
{
}

void Point2Plane::Instantiate(Point2PlaneInitializer& init)
{
    // We are using the Hessian Normal Form representation of a plane, cf.:
    // http://mathworld.wolfram.com/HessianNormalForm.html
    auto plane = PlaneGeometryInitializer(init.Plane);
    normal_ = plane.Normal / plane.Normal.norm();
    origin_ = plane.Origin;

    if (debug_) HIGHLIGHT_NAMED("Point2Plane", "Plane Origin: " << origin_.transpose() << " and Normal: " << normal_.transpose());

    // TODO(wxm): If debug and ROS, set up publisher
}

void Point2Plane::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != Kinematics.Phi.rows()) throw_named("Wrong size of phi!");

    for (int i = 0; i < Kinematics.Phi.rows(); i++)
    {
        const auto& point = Eigen::Map<const Eigen::Vector3d>(Kinematics.Phi(i).p.data);
        phi(i) = std::abs(normal_.dot(point - origin_));
    }
}

void Point2Plane::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != Kinematics.Phi.rows()) throw_named("Wrong size of phi!");
    if (J.rows() != Kinematics.J.rows() || J.cols() != Kinematics.J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics.J(0).data.cols());

    J.setZero();

    for (int i = 0; i < Kinematics.Phi.rows(); i++)
    {
        const auto& point = Eigen::Map<const Eigen::Vector3d>(Kinematics.Phi(i).p.data);
        double signed_distance = normal_.dot(point - origin_);
        phi(i) = std::abs(signed_distance);

        // Direction from point to plane is following the vector from the point to the closest point on the plane, i.e.:
        Eigen::Vector3d projected_point = point - signed_distance * normal_;
        Eigen::Vector3d projection_dir = projected_point - point;

        for (int j = 0; j < J.cols(); j++)
        {
            J(i, j) = -projection_dir.dot(Eigen::Map<const Eigen::Vector3d>(Kinematics.J[i].getColumn(j).vel.data)) / phi(i);
        }
    }
}

int Point2Plane::taskSpaceDim()
{
    return Kinematics.Phi.rows();
}
}  // namespace exotica
