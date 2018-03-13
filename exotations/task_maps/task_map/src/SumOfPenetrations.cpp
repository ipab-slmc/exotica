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

#include "task_map/SumOfPenetrations.h"

REGISTER_TASKMAP_TYPE("SumOfPenetrations",
                      exotica::SumOfPenetrations);

namespace exotica
{
SumOfPenetrations::SumOfPenetrations() {}
void SumOfPenetrations::update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    phi.setZero();
    Eigen::MatrixXd J;

    // Get all world collision links, then iterate through them
    std::vector<CollisionProxy> proxies = cscene_->getCollisionDistance(robotLinks_, check_self_collision_);
    double& d = phi(0);
    for (const auto& proxy : proxies)
    {
        bool isRobotToRobot = (proxy.e1->isRobotLink || proxy.e1->ClosestRobotLink.lock()) && (proxy.e2->isRobotLink || proxy.e2->ClosestRobotLink.lock());
        double& margin = isRobotToRobot ? robot_margin_ : world_margin_;
        if (proxy.distance < margin)
        {
            if (proxy.distance < 0)
            {
                d += std::abs(proxy.distance);
            }
            else
            {
                d += std::abs(proxy.distance - margin);
            }
        }
    }
}

void SumOfPenetrations::update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi,
                               Eigen::MatrixXdRef J)
{
    throw_pretty("Not implemented");
    // J.setZero();
    // update(x, phi);
}

void SumOfPenetrations::Instantiate(
    SumOfPenetrationsInitializer& init)
{
    init_ = init;
}

void SumOfPenetrations::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void SumOfPenetrations::Initialize()
{
    cscene_ = scene_->getCollisionScene();
    world_margin_ = init_.WorldMargin;
    robot_margin_ = init_.RobotMargin;
    check_self_collision_ = init_.CheckSelfCollision;

    HIGHLIGHT_NAMED("Sum of Penetrations",
                    "World Margin: " << world_margin_ << " Robot Margin: " << robot_margin_);

    // Get names of all controlled joints and their corresponding child links
    robotLinks_ = scene_->getControlledLinkNames();
    // std::cout << "Robot links: ";
    // for (auto& link : robotLinks_) std::cout << link << ", ";
    // std::cout << std::endl;
}

int SumOfPenetrations::taskSpaceDim() { return dim_; }
}
