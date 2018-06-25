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

#include "task_map/SphereCollision.h"

REGISTER_TASKMAP_TYPE("SphereCollision", exotica::SphereCollision);

namespace exotica
{
SphereCollision::SphereCollision()
{
}

void SphereCollision::Instantiate(SphereCollisionInitializer& init)
{
    eps = 1.0 / init.Precision;
    for (int i = 0; i < init.EndEffector.size(); i++)
    {
        SphereInitializer sphere(init.EndEffector[i]);
        groups[sphere.Group].push_back(i);
        radiuses.push_back(sphere.Radius);
        visualization_msgs::Marker mrk;
        mrk.action = visualization_msgs::Marker::ADD;
        mrk.header.frame_id = init.ReferenceFrame;
        mrk.id = i;
        mrk.type = visualization_msgs::Marker::SPHERE;
        mrk.scale.x = mrk.scale.y = mrk.scale.z = sphere.Radius * 2;
        debug_msg.markers.push_back(mrk);
    }
    for (auto& it : groups)
    {
        std_msgs::ColorRGBA col = randomColor();
        col.a = init.Alpha;
        for (int i : it.second)
        {
            debug_msg.markers[i].color = col;
            debug_msg.markers[i].ns = it.first;
        }
    }
    debug = init.Debug;
    if (debug)
    {
        debug_pub = Server::advertise<visualization_msgs::MarkerArray>(ns_ + "/CollisionSpheres", 1, true);
    }
}

double SphereCollision::distance(const KDL::Frame& effA, const KDL::Frame& effB, double rA, double rB)
{
    return 1.0 / (1.0 + exp(5.0 * eps * ((effA.p - effB.p).Norm() - rA - rB)));
}

Eigen::VectorXd SphereCollision::Jacobian(const KDL::Frame& effA, const KDL::Frame& effB, const KDL::Jacobian& jacA, const KDL::Jacobian& jacB, double rA, double rB)
{
    int n = jacA.columns();
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd eA(3);
    Eigen::VectorXd eB(3);
    for (int i = 0; i < n; i++)
    {
        eA << effA.p[0], effA.p[1], effA.p[2];
        eB << effB.p[0], effB.p[1], effB.p[2];
        ret(i) = -(double)((jacA.data.col(i).head(3) - jacB.data.col(i).head(3)).adjoint() * (eA - eB)) / (effA.p - effB.p).Norm();
    }
    return ret;
}

void SphereCollision::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != taskSpaceDim()) throw_named("Wrong size of phi!");
    phi.setZero();

    auto Aend = groups.end()--;
    auto Bend = groups.end();
    int phiI = 0;
    for (auto A = groups.begin(); A != Aend; A++)
    {
        for (auto B = std::next(A); B != Bend; B++)
        {
            for (int ii = 0; ii < A->second.size(); ii++)
            {
                for (int jj = 0; jj < B->second.size(); jj++)
                {
                    int i = A->second[ii];
                    int j = B->second[jj];
                    phi(phiI) += distance(Kinematics[0].Phi(i), Kinematics[0].Phi(j), radiuses[i], radiuses[j]);
                }
            }
            phiI++;
        }
    }

    if (debug)
    {
        for (int i = 0; i < debug_msg.markers.size(); i++)
        {
            debug_msg.markers[i].pose.position.x = Kinematics[0].Phi(i).p[0];
            debug_msg.markers[i].pose.position.y = Kinematics[0].Phi(i).p[1];
            debug_msg.markers[i].pose.position.z = Kinematics[0].Phi(i).p[2];
        }
        debug_pub.publish(debug_msg);
    }
}

void SphereCollision::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != taskSpaceDim()) throw_named("Wrong size of phi!");
    if (J.rows() != taskSpaceDim() || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());
    phi.setZero();
    J.setZero();

    auto Aend = groups.end()--;
    auto Bend = groups.end();
    int phiI = 0;
    for (auto A = groups.begin(); A != Aend; A++)
    {
        for (auto B = std::next(A); B != Bend; B++)
        {
            for (int ii = 0; ii < A->second.size(); ii++)
            {
                for (int jj = 0; jj < B->second.size(); jj++)
                {
                    int i = A->second[ii];
                    int j = B->second[jj];
                    phi(phiI) += distance(Kinematics[0].Phi(i), Kinematics[0].Phi(j), radiuses[i], radiuses[j]);
                    J.row(phiI) += Jacobian(Kinematics[0].Phi(i), Kinematics[0].Phi(j), Kinematics[0].J(i), Kinematics[0].J(j), radiuses[i], radiuses[j]);
                }
            }
            phiI++;
        }
    }

    if (debug)
    {
        for (int i = 0; i < debug_msg.markers.size(); i++)
        {
            debug_msg.markers[i].pose.position.x = Kinematics[0].Phi(i).p[0];
            debug_msg.markers[i].pose.position.y = Kinematics[0].Phi(i).p[1];
            debug_msg.markers[i].pose.position.z = Kinematics[0].Phi(i).p[2];
        }
        debug_pub.publish(debug_msg);
    }
}

int SphereCollision::taskSpaceDim()
{
    return groups.size() * (groups.size() - 1) / 2;
}
}
