//
// Copyright (c) 2017, Wolfgang Merkt
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

#include "exotica_core_task_maps/smooth_collision_distance.h"

REGISTER_TASKMAP_TYPE("SmoothCollisionDistance", exotica::SmoothCollisionDistance);

namespace exotica
{
void SmoothCollisionDistance::Update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    Eigen::MatrixXd J(dim_, x.size());
    UpdateInternal(x, phi, J, false);
}

void SmoothCollisionDistance::Update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi,
                                     Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    J.setZero();
    UpdateInternal(x, phi, J, true);
}

void SmoothCollisionDistance::UpdateInternal(Eigen::VectorXdRefConst x,
                                             Eigen::VectorXdRef phi,
                                             Eigen::MatrixXdRef J,
                                             bool updateJacobian)
{
    if (!scene_->AlwaysUpdatesCollisionScene())
        cscene_->UpdateCollisionObjectTransforms();

    //  1) Create vector to store CollisionProxy
    std::vector<CollisionProxy> proxies;

    //  2) For each robot link, check against each robot link
    if (check_self_collision_)
    {
        AppendVector(proxies, cscene_->GetRobotToRobotCollisionDistance(robot_margin_));
    }

    //  3) For each robot link, check against each environment link
    AppendVector(proxies, cscene_->GetRobotToWorldCollisionDistance(world_margin_));

    //  4) Compute d, J
    double& d = phi(0);
    {
        KDL::Frame arel, brel;
        Eigen::MatrixXd J_a(6, scene_->GetKinematicTree().GetNumControlledJoints()), J_b(6, scene_->GetKinematicTree().GetNumControlledJoints());
        for (const auto& proxy : proxies)
        {
            bool is_robot_to_robot = (proxy.e1 != nullptr && proxy.e2 != nullptr) && (proxy.e1->is_robot_link || proxy.e1->closest_robot_link.lock()) && (proxy.e2->is_robot_link || proxy.e2->closest_robot_link.lock());
            double& margin = is_robot_to_robot ? robot_margin_ : world_margin_;

            if (proxy.distance < margin)
            {
                // Cost
                d += std::pow((1. - proxy.distance / margin), linear_ ? 1 : 2);

                if (updateJacobian)
                {
                    // Jacobian
                    if (proxy.e1 != nullptr)
                    {
                        arel = KDL::Frame(proxy.e1->frame.Inverse(KDL::Vector(proxy.contact1.x(), proxy.contact1.y(), proxy.contact1.z())));
                        J_a = scene_->GetKinematicTree().Jacobian(proxy.e1, arel, nullptr, KDL::Frame());
                    }
                    else
                    {
                        arel = arel.Identity();
                        J_a.setZero();
                    }

                    if (proxy.e2 != nullptr)
                    {
                        brel = KDL::Frame(proxy.e2->frame.Inverse(KDL::Vector(proxy.contact2.x(), proxy.contact2.y(), proxy.contact2.z())));
                        J_b = scene_->GetKinematicTree().Jacobian(proxy.e2, brel, nullptr, KDL::Frame());
                    }
                    else
                    {
                        brel = brel.Identity();
                        J_b.setZero();
                    }

                    if (!linear_)
                    {
                        J += (2. / (margin * margin)) * (proxy.normal1.transpose() * J_a.topRows<3>());
                        J -= (2. / (margin * margin)) * (proxy.normal1.transpose() * J_b.topRows<3>());
                    }
                    else
                    {
                        J += 1 / margin * (proxy.normal1.transpose() * J_a.topRows<3>());
                        J -= 1 / margin * (proxy.normal1.transpose() * J_b.topRows<3>());
                    }
                }
            }
        }
    }
}

void SmoothCollisionDistance::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void SmoothCollisionDistance::Initialize()
{
    cscene_ = scene_->GetCollisionScene();
    world_margin_ = parameters_.WorldMargin;
    robot_margin_ = parameters_.RobotMargin;
    linear_ = parameters_.Linear;
    check_self_collision_ = parameters_.CheckSelfCollision;

    if (robot_margin_ == 0.0 || world_margin_ == 0.0) ThrowPretty("Setting the margin to zero is a bad idea. It will NaN.");

    if (debug_) HIGHLIGHT_NAMED("Smooth Collision Distance",
                                "World Margin: " << world_margin_ << " Robot Margin: " << robot_margin_ << "\t Linear: " << linear_);
}

int SmoothCollisionDistance::TaskSpaceDim() { return dim_; }
}  // namespace exotica
