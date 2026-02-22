//
// Copyright (c) 2018, Wolfgang Merkt
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

#include "exotica_core_task_maps/sum_of_penetrations.h"

REGISTER_TASKMAP_TYPE("SumOfPenetrations", exotica::SumOfPenetrations);

namespace exotica
{
void SumOfPenetrations::Update(Eigen::VectorXdRefConst x,
                               Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    Eigen::MatrixXd J;

    //  1) Create vector to store CollisionProxy
    std::vector<CollisionProxy> proxies;

    //  2) For each robot link, check against each robot link
    if (check_self_collision_)
    {
        AppendVector(proxies, cscene_->GetRobotToRobotCollisionDistance(robot_margin_));
    }

    //  3) For each robot link, check against each environment link
    AppendVector(proxies, cscene_->GetRobotToWorldCollisionDistance(world_margin_));

    double& d = phi(0);
    for (const auto& proxy : proxies)
    {
        bool is_robot_to_robot = (proxy.e1->is_robot_link || proxy.e1->closest_robot_link.lock()) && (proxy.e2->is_robot_link || proxy.e2->closest_robot_link.lock());
        double& margin = is_robot_to_robot ? robot_margin_ : world_margin_;
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

void SumOfPenetrations::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void SumOfPenetrations::Initialize()
{
    cscene_ = scene_->GetCollisionScene();
    world_margin_ = parameters_.WorldMargin;
    robot_margin_ = parameters_.RobotMargin;
    check_self_collision_ = parameters_.CheckSelfCollision;

    HIGHLIGHT_NAMED("Sum of Penetrations",
                    "World Margin: " << world_margin_ << " Robot Margin: " << robot_margin_);
}

int SumOfPenetrations::TaskSpaceDim() { return dim_; }
}  // namespace exotica
