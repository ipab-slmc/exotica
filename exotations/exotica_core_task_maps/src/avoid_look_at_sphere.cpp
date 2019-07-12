//
// Copyright (c) 2019, Christopher E. Mower
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

#include <exotica_core/server.h>
#include <exotica_core_task_maps/avoid_look_at_sphere.h>

REGISTER_TASKMAP_TYPE("AvoidLookAtSphere", exotica::AvoidLookAtSphere);

namespace exotica
{
void AvoidLookAtSphere::UpdateAsCostWithoutJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    for (int i = 0; i < n_objects_; ++i)
    {
        const double frac = Eigen::Map<Eigen::Vector2d>(kinematics[0].Phi(i).p.data).squaredNorm() / radii_squared_(i);
        if (frac < 1.0)
        {
            phi(i) = 1.0 - 2.0 * frac + frac * frac;
        }
        else
        {
            phi(i) = 0.0;
        }
    }
}

void AvoidLookAtSphere::UpdateAsInequalityConstraintWithoutJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    for (int i = 0; i < n_objects_; ++i) phi(i) = radii_squared_(i) - Eigen::Map<Eigen::Vector2d>(kinematics[0].Phi(i).p.data).squaredNorm();
}

void AvoidLookAtSphere::UpdateAsCostWithJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    for (int i = 0; i < n_objects_; ++i)
    {
        Eigen::Vector2d o = Eigen::Map<Eigen::Vector2d>(kinematics[0].Phi(i).p.data);
        const double frac = o.squaredNorm() / radii_squared_(i);
        if (frac < 1.0)
        {
            phi(i) = 1.0 - 2.0 * frac + frac * frac;
            for (int j = 0; j < jacobian.cols(); ++j) jacobian(i, j) = -4.0 * (1.0 - frac) * kinematics[0].jacobian[i].data.topRows<2>().col(j).dot(o) / radii_squared_(i);
        }
        else
        {
            phi(i) = 0.0;
            jacobian.row(i).setZero();
        }
    }
}

void AvoidLookAtSphere::UpdateAsInequalityConstraintWithJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    for (int i = 0; i < n_objects_; ++i)
    {
        Eigen::Vector2d o = Eigen::Map<Eigen::Vector2d>(kinematics[0].Phi(i).p.data);
        const double d = o.norm();
        phi(i) = radii_squared_(i) - d * d;
        for (int j = 0; j < jacobian.cols(); ++j) jacobian(i, j) = -2.0 * kinematics[0].jacobian[i].data.topRows<2>().col(j).dot(o);
    }
}

void AvoidLookAtSphere::PublishObjectsAsMarkerArray()
{
    const ros::Time t = ros::Time::now();
    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(n_objects_);
    for (int i = 0; i < n_objects_; ++i)
    {
        visualization_msgs::Marker m;
        m.header.stamp = t;
        m.header.frame_id = "exotica/" + frames_[i].frame_B_link_name;
        m.id = i;
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::SPHERE;
        m.scale.x = diameter_(i);
        m.scale.y = diameter_(i);
        m.scale.z = diameter_(i);
        Eigen::Vector4d q = GetRotationAsVector(kinematics[0].Phi(i), RotationType::QUATERNION);
        m.pose.position.x = kinematics[0].Phi(i).p.data[0];
        m.pose.position.y = kinematics[0].Phi(i).p.data[1];
        m.pose.position.z = kinematics[0].Phi(i).p.data[2];
        m.pose.orientation.x = q[0];
        m.pose.orientation.y = q[1];
        m.pose.orientation.z = q[2];
        m.pose.orientation.w = q[3];
        m.color.r = 1.0;
        m.color.a = 1.0;
        ma.markers.emplace_back(m);
    }
    pub_markers_.publish(ma);
}

void AvoidLookAtSphere::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    (this->*UpdateTaskMapWithoutJacobian)(x, phi);
    if (debug_ && Server::IsRos()) PublishObjectsAsMarkerArray();
}

void AvoidLookAtSphere::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    (this->*UpdateTaskMapWithJacobian)(x, phi, jacobian);
    if (debug_ && Server::IsRos()) PublishObjectsAsMarkerArray();
}

void AvoidLookAtSphere::Instantiate(const AvoidLookAtSphereInitializer& init)
{
    parameters_ = init;
    n_objects_ = frames_.size();
    diameter_.resize(n_objects_, 1);
    radii_squared_.resize(n_objects_, 1);

    for (int i = 0; i < n_objects_; ++i)
    {
        SphereInitializer sphere(init.EndEffector[i]);
        diameter_(i) = 2.0 * sphere.Radius;
        radii_squared_(i) = sphere.Radius * sphere.Radius;
    }

    if (init.UseAsCost)
    {
        UpdateTaskMapWithoutJacobian = &AvoidLookAtSphere::UpdateAsCostWithoutJacobian;
        UpdateTaskMapWithJacobian = &AvoidLookAtSphere::UpdateAsCostWithJacobian;
    }
    else
    {
        UpdateTaskMapWithoutJacobian = &AvoidLookAtSphere::UpdateAsInequalityConstraintWithoutJacobian;
        UpdateTaskMapWithJacobian = &AvoidLookAtSphere::UpdateAsInequalityConstraintWithJacobian;
    }

    if (debug_ && Server::IsRos())
    {
        pub_markers_ = Server::Advertise<visualization_msgs::MarkerArray>("avoid_look_at_sphere_objects", 1, true);
        visualization_msgs::Marker md;  // delete previous markers
        md.action = visualization_msgs::Marker::DELETEALL;
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(md);
        pub_markers_.publish(ma);
    }
}

int AvoidLookAtSphere::TaskSpaceDim()
{
    return n_objects_;
}
}  // namespace exotica
