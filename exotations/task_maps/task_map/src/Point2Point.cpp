/*
 *      Author: Christopher E. Mower
 *
 * Copyright (c) 2018, University Of Edinburgh
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

#include "task_map/Point2Point.h"

REGISTER_TASKMAP_TYPE("Point2Point", exotica::Point2Point);

namespace exotica
{
Point2Point::Point2Point() {}  // <<--- never ever forget the constructor!!
Eigen::Vector3d Point2Point::getPoint()
{
    return point_;
}

void Point2Point::setPoint(Eigen::Vector3d point)
{
    point_ = point;
}

void Point2Point::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    phi(0) = 0.0;
}

Eigen::Vector3d Point2Point::getEffPosition(int i)
{
    return Eigen::Map<Eigen::Vector3d>(Kinematics[0].Phi(i).p.data);
}

Eigen::Matrix3d Point2Point::getEffRotation(double roll, double pitch, double yaw)
{
    // Computes the rotation matrix uing rpy as returned by getEffRotation(i)
    // This is unused, here only for reference.

    Eigen::Matrix3d R;
    double cr, sr;  // cos/sin(roll)
    double cp, sp;  // cos/sin(roll)
    double cy, sy;  // cos/sin(roll)

    cr = std::cos(roll);
    sr = std::sin(roll);

    cp = std::cos(pitch);
    sp = std::sin(pitch);

    cy = std::cos(yaw);
    sy = std::sin(yaw);

    // Row 1
    R(0, 0) = cr * cp;
    R(0, 1) = cr * sp * sy - sr * cy;
    R(0, 2) = cr * sp * cy + sr * sy;

    // Row 2
    R(1, 0) = sr * cp;
    R(1, 1) = sr * sp * sy + cr * cy;
    R(1, 2) = sr * sp * cy - cr * sy;

    // Row 3
    R(2, 0) = -sp;
    R(2, 1) = cp * sy;
    R(2, 2) = cp * cy;

    return R;
}

Eigen::Vector3d Point2Point::getApproachVector(double roll, double pitch, double yaw)
{
    // Computes approach vector using rpy, as getApproachVector(i) does.
    // This is unused and left only for reference.

    Eigen::Vector3d ahat;

    double cr, sr;  // cos/sin(roll)
    double cp, sp;  // cos/sin(roll)
    double cy, sy;  // cos/sin(roll)

    // This is col(0) of rotation matrix as returned by Kinematics

    cr = std::cos(roll);
    sr = std::sin(roll);

    cp = std::cos(pitch);
    sp = std::sin(pitch);

    cy = std::cos(yaw);
    sy = std::sin(yaw);

    ahat(0) = cr * cp;
    ahat(1) = sr * cp;
    ahat(2) = -sp;

    return ahat;
}

Eigen::Vector3d Point2Point::getApproachVector(int i)
{
    return getEffRotation(i).col(0);  // approach vector is x axis or exotica/Frame0Alwr_arm_6_link!
}

Eigen::Matrix3d Point2Point::getEffRotation(int i)
{
    return Eigen::Map<Eigen::Matrix3d>(Kinematics[0].Phi(i).M.data).transpose();  // note, exotica/kdl returns transpose of matrix required
}

Eigen::Matrix3d Point2Point::getApproachVectorJacobian(double roll, double pitch, double yaw)
{
    // Approach vector jacobian (aJ)
    //
    // Let Phi = (r, p, y) be rpy euler angles
    //     ahat = ahat(Phi) be approach vector of eff
    //
    // Then aJ = d ahat(Phi) / d Phi

    Eigen::Matrix3d J;

    double cr, sr;  // cos/sin(roll)
    double cp, sp;  // cos/sin(roll)
    double cy, sy;  // cos/sin(roll)

    cr = std::cos(roll);
    sr = std::sin(roll);

    cp = std::cos(pitch);
    sp = std::sin(pitch);

    cy = std::cos(yaw);
    sy = std::sin(yaw);

    // Row 1
    J(0, 0) = -sr * cp;
    J(0, 1) = -cr * sp;

    // Row 2
    J(1, 0) = cr * cp;
    J(1, 1) = -sr * sp;

    // Row 3
    J(2, 1) = -cp;

    return J;
}

void Point2Point::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    // Collect rpy angles
    double roll, pitch, yaw;
    Kinematics[0].Phi(0).M.GetRPY(roll, pitch, yaw);

    // Compute eff postion and approach vector
    Eigen::Vector3d e = getEffPosition(0);
    Eigen::Vector3d ahat = getApproachVector(0);

    // Setup
    Eigen::Vector3d eap = e + ahat - point_;
    Eigen::Vector3d L = point_ - e;
    Eigen::Vector3d K = ahat.cross(eap);
    Eigen::Matrix3d J_rpy = getApproachVectorJacobian(roll, pitch, yaw);
    double LL = L.squaredNorm();
    double LL2 = LL * LL;
    double KK = K.squaredNorm();
    Eigen::MatrixXd J_q = Kinematics[0].J(0).data;

    // Compute phi
    phi(0) = KK / LL;

    // Compute J
    for (int i = 0; i < N; i++)
    {
        // Setup
        Eigen::Vector3d ed = J_q.col(i).topRows(3);
        Eigen::Vector3d ahatd = J_rpy * J_q.col(i).bottomRows(3);
        Eigen::Vector3d Kd = ahatd.cross(eap) + ahat.cross(ed + ahatd);
        Eigen::Vector3d Ld = -ed;

        J(0, i) = 2.0 * (Kd.dot(K) * LL - KK * Ld.dot(L)) / LL2;
    }
}

void Point2Point::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void Point2Point::Initialize()
{
    N = scene_->getSolver().getNumControlledJoints();
}

void Point2Point::Instantiate(Point2PointInitializer &init)
{
    point_ = init.Point;
}

int Point2Point::taskSpaceDim()
{
    return 1;
}
}  // namespace exotica
