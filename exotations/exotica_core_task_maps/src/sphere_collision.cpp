//
// Copyright (c) 2018, University of Edinburgh
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
#include <exotica_core_task_maps/sphere_collision.h>

REGISTER_TASKMAP_TYPE("SphereCollision", exotica::SphereCollision);

namespace exotica
{
SphereCollision::SphereCollision() = default;
SphereCollision::~SphereCollision() = default;

void SphereCollision::Instantiate(SphereCollisionInitializer& init)
{
    eps_ = 1.0 / init.Precision;
    for (int i = 0; i < init.EndEffector.size(); ++i)
    {
        SphereInitializer sphere(init.EndEffector[i]);
        groups_[sphere.Group].push_back(i);
        radiuses_.push_back(sphere.Radius);
        visualization_msgs::Marker mrk;
        mrk.action = visualization_msgs::Marker::ADD;
        mrk.header.frame_id = init.ReferenceFrame;
        mrk.id = i;
        mrk.type = visualization_msgs::Marker::SPHERE;
        mrk.scale.x = mrk.scale.y = mrk.scale.z = sphere.Radius * 2;
        debug_msg_.markers.push_back(mrk);
    }
    for (auto& it : groups_)
    {
        std_msgs::ColorRGBA col = RandomColor();
        col.a = init.Alpha;
        for (int i : it.second)
        {
            debug_msg_.markers[i].color = col;
            debug_msg_.markers[i].ns = it.first;
        }
    }

    if (debug_)
    {
        debug_pub_ = Server::Advertise<visualization_msgs::MarkerArray>(ns_ + "/CollisionSpheres", 1, true);
    }
}

double SphereCollision::Distance(const KDL::Frame& eff_A, const KDL::Frame& eff_B, double r_A, double r_B)
{
    return 1.0 / (1.0 + exp(5.0 * eps_ * ((eff_A.p - eff_B.p).Norm() - r_A - r_B)));
}

Eigen::VectorXd SphereCollision::Jacobian(const KDL::Frame& eff_A, const KDL::Frame& eff_B, const KDL::Jacobian& jacA, const KDL::Jacobian& jacB, double r_A, double r_B)
{
    int n = jacA.columns();
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd eA(3);
    Eigen::VectorXd eB(3);
    for (int i = 0; i < n; ++i)
    {
        eA << eff_A.p[0], eff_A.p[1], eff_A.p[2];
        eB << eff_B.p[0], eff_B.p[1], eff_B.p[2];
        ret(i) = -static_cast<double>((jacA.data.col(i).head(3) - jacB.data.col(i).head(3)).adjoint() * (eA - eB)) / (eff_A.p - eff_B.p).Norm();
    }
    return ret;
}

void SphereCollision::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    phi.setZero();

    auto Aend = groups_.end()--;
    auto Bend = groups_.end();
    int phiI = 0;
    for (auto A = groups_.begin(); A != Aend; ++A)
    {
        for (auto B = std::next(A); B != Bend; ++B)
        {
            for (int ii = 0; ii < A->second.size(); ++ii)
            {
                for (int jj = 0; jj < B->second.size(); ++jj)
                {
                    int i = A->second[ii];
                    int j = B->second[jj];
                    phi(phiI) += Distance(kinematics[0].Phi(i), kinematics[0].Phi(j), radiuses_[i], radiuses_[j]);
                }
            }
            ++phiI;
        }
    }

    if (debug_)
    {
        for (int i = 0; i < debug_msg_.markers.size(); ++i)
        {
            debug_msg_.markers[i].pose.position.x = kinematics[0].Phi(i).p[0];
            debug_msg_.markers[i].pose.position.y = kinematics[0].Phi(i).p[1];
            debug_msg_.markers[i].pose.position.z = kinematics[0].Phi(i).p[2];
        }
        debug_pub_.publish(debug_msg_);
    }
}

void SphereCollision::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != TaskSpaceDim() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    phi.setZero();
    jacobian.setZero();

    auto Aend = groups_.end()--;
    auto Bend = groups_.end();
    int phiI = 0;
    for (auto A = groups_.begin(); A != Aend; ++A)
    {
        for (auto B = std::next(A); B != Bend; ++B)
        {
            for (int ii = 0; ii < A->second.size(); ++ii)
            {
                for (int jj = 0; jj < B->second.size(); ++jj)
                {
                    int i = A->second[ii];
                    int j = B->second[jj];
                    phi(phiI) += Distance(kinematics[0].Phi(i), kinematics[0].Phi(j), radiuses_[i], radiuses_[j]);
                    jacobian.row(phiI) += Jacobian(kinematics[0].Phi(i), kinematics[0].Phi(j), kinematics[0].jacobian(i), kinematics[0].jacobian(j), radiuses_[i], radiuses_[j]);
                }
            }
            ++phiI;
        }
    }

    if (debug_)
    {
        for (int i = 0; i < debug_msg_.markers.size(); ++i)
        {
            debug_msg_.markers[i].pose.position.x = kinematics[0].Phi(i).p[0];
            debug_msg_.markers[i].pose.position.y = kinematics[0].Phi(i).p[1];
            debug_msg_.markers[i].pose.position.z = kinematics[0].Phi(i).p[2];
        }
        debug_pub_.publish(debug_msg_);
    }
}

int SphereCollision::TaskSpaceDim()
{
    return groups_.size() * (groups_.size() - 1) / 2;
}
}
