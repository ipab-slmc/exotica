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

#include "task_map/CollisionDistance.h"

REGISTER_TASKMAP_TYPE("CollisionDistance", exotica::CollisionDistance);

namespace exotica
{
CollisionDistance::CollisionDistance() {}
void CollisionDistance::update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    phi.setZero();
    Eigen::MatrixXd J;
    update(x, phi, J, false);
}

void CollisionDistance::update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi,
                               Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    phi.setZero();
    J.setZero();
    update(x, phi, J, true);
}

void CollisionDistance::update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi,
                               Eigen::MatrixXdRef J,
                               bool updateJacobian)
{
    cscene_->updateCollisionObjectTransforms();

    // For all robot links: Get all collision distances, sort by distance, and process the closest.
    for (unsigned int i = 0; i < dim_; i++)
    {
        // std::vector<CollisionProxy> proxies = cscene_->getCollisionDistance(scene_->getControlledLinkToCollisionLinkMap()[robotLinks_[i]], check_self_collision_);  //, false);
        std::vector<CollisionProxy> proxies = cscene_->getCollisionDistance(robotLinks_[i], check_self_collision_);
        if (proxies.size() == 0)
        {
            phi(i) = 0;
            J.row(i).setZero();
            continue;
        }

        CollisionProxy& closest_proxy = closestProxies_[i];
        closest_proxy.distance = std::numeric_limits<double>::max();
        for (const auto& tmp_proxy : proxies)
        {
            const bool isRobotToRobot = (tmp_proxy.e1->isRobotLink || tmp_proxy.e1->ClosestRobotLink.lock()) && (tmp_proxy.e2->isRobotLink || tmp_proxy.e2->ClosestRobotLink.lock());
            const double& margin = isRobotToRobot ? robot_margin_ : world_margin_;
            if ((tmp_proxy.distance - margin) < closest_proxy.distance)
            {
                closest_proxy = tmp_proxy;
                closest_proxy.distance -= margin;
            }
        }

        phi(i) = closest_proxy.distance;

        if (updateJacobian)
        {
            KDL::Frame arel = KDL::Frame(closest_proxy.e1->Frame.Inverse(KDL::Vector(
                closest_proxy.contact1(0), closest_proxy.contact1(1), closest_proxy.contact1(2))));
            KDL::Frame brel = KDL::Frame(closest_proxy.e2->Frame.Inverse(KDL::Vector(
                closest_proxy.contact2(0), closest_proxy.contact2(1), closest_proxy.contact2(2))));

            Eigen::MatrixXd tmpJ = scene_->getSolver().Jacobian(
                closest_proxy.e1, arel, nullptr, KDL::Frame());
            J.row(i) += (closest_proxy.normal1.transpose() * tmpJ);
            tmpJ = scene_->getSolver().Jacobian(closest_proxy.e2, brel, nullptr,
                                                KDL::Frame());
            J.row(i) -= (closest_proxy.normal1.transpose() * tmpJ);
        }
    }
}

void CollisionDistance::Instantiate(
    CollisionDistanceInitializer& init)
{
    init_ = init;
}

void CollisionDistance::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void CollisionDistance::Initialize()
{
    cscene_ = scene_->getCollisionScene();
    check_self_collision_ = init_.CheckSelfCollision;
    world_margin_ = init_.WorldMargin;
    robot_margin_ = init_.RobotMargin;

    // Get names of all controlled joints and their corresponding child links
    robotLinks_ = scene_->getControlledLinkNames();
    dim_ = static_cast<unsigned int>(robotLinks_.size());
    closestProxies_.assign(dim_, CollisionProxy());
    if (debug_)
    {
        HIGHLIGHT_NAMED("Collision Distance", "Dimension: " << dim_
                                                            << " - CheckSelfCollision: " << check_self_collision_
                                                            << "World Margin: " << world_margin_
                                                            << " Robot Margin: " << robot_margin_);
    }
}

int CollisionDistance::taskSpaceDim() { return dim_; }
}
