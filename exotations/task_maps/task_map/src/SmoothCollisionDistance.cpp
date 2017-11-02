/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2017, Wolfgang Merkt
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

#include "task_map/SmoothCollisionDistance.h"

REGISTER_TASKMAP_TYPE("SmoothCollisionDistance",
                      exotica::SmoothCollisionDistance);

namespace exotica
{
SmoothCollisionDistance::SmoothCollisionDistance() {}
void SmoothCollisionDistance::update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi)
{
    throw_pretty("I didn't implement that yet");
}

void SmoothCollisionDistance::update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi,
                                     Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    phi.setZero();
    J.setZero();

    // Get all world collision links, then iterate through them
    {
        std::vector<CollisionProxy> proxies = cscene_->getCollisionDistance(robotLinks_, check_self_collision_);
        double& d = phi(0);
        for (const auto& proxy : proxies)
        {
            bool isRobotToRobot = (proxy.e1->isRobotLink || proxy.e1->ClosestRobotLink) && (proxy.e2->isRobotLink || proxy.e2->ClosestRobotLink);
            double& margin = isRobotToRobot ? robot_margin_ : world_margin_;
            if (proxy.distance < margin)
            {
                // Cost
                d += std::pow((1. - proxy.distance / margin), linear_ ? 1 : 2);

                // Jacobian
                KDL::Frame arel = KDL::Frame(proxy.e1->Frame.Inverse(KDL::Vector(
                    proxy.contact1(0), proxy.contact1(1), proxy.contact1(2))));
                KDL::Frame brel = KDL::Frame(proxy.e2->Frame.Inverse(KDL::Vector(
                    proxy.contact2(0), proxy.contact2(1), proxy.contact2(2))));

                // HIGHLIGHT_NAMED("A-Rel", arel.p);
                // HIGHLIGHT_NAMED("B-Rel", brel.p);

                if (!linear_)
                {
                    Eigen::MatrixXd tmpJ = scene_->getSolver().Jacobian(
                        proxy.e1, arel, nullptr, KDL::Frame());
                    J -= (2. * d) / margin * (proxy.normal1.transpose() * tmpJ);
                    tmpJ = scene_->getSolver().Jacobian(proxy.e2, brel, nullptr,
                                                        KDL::Frame());
                    J += (2. * d) / margin * (proxy.normal1.transpose() * tmpJ);
                }
                else
                {
                    throw_pretty("Noooooo");
                    Eigen::MatrixXd tmpJ = scene_->getSolver().Jacobian(
                        proxy.e1, arel, nullptr, KDL::Frame());
                    J -= 1 / margin * (proxy.normal1.transpose() * tmpJ);
                    tmpJ = scene_->getSolver().Jacobian(proxy.e2, brel, nullptr,
                                                        KDL::Frame());
                    J += 1 / margin * (proxy.normal1.transpose() * tmpJ);
                }
            }
        }
    }
}

void SmoothCollisionDistance::Instantiate(
    SmoothCollisionDistanceInitializer& init)
{
    init_ = init;
}

void SmoothCollisionDistance::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void SmoothCollisionDistance::Initialize()
{
    cscene_ = scene_->getCollisionScene();
    world_margin_ = init_.WorldMargin;
    robot_margin_ = init_.RobotMargin;
    linear_ = init_.Linear;
    check_self_collision_ = init_.CheckSelfCollision;

    HIGHLIGHT_NAMED("Smooth Collision Distance",
                    "World Margin: " << world_margin_ << " Robot Margin: " << robot_margin_ << "\t Linear: " << linear_);

    // Get names of all controlled joints and their corresponding child links
    robotLinks_ = scene_->getControlledLinkNames();
}

int SmoothCollisionDistance::taskSpaceDim() { return dim_; }
}
