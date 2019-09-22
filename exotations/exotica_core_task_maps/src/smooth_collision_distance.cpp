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
    Eigen::MatrixXd J(dim_, robot_joints_.size());
    Update(x, phi, J, false);
}

void SmoothCollisionDistance::Update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi,
                                     Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    J.setZero();
    Update(x, phi, J, true);
}

void SmoothCollisionDistance::Update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi,
                                     Eigen::MatrixXdRef J,
                                     bool updateJacobian)
{
    if (!scene_->AlwaysUpdatesCollisionScene())
        cscene_->UpdateCollisionObjectTransforms();

    double& d = phi(0);

    for (const auto& joint : robot_joints_)
    {
        // Get all world collision links, then iterate through them
        std::vector<CollisionProxy> proxies = cscene_->GetCollisionDistance(controlled_joint_to_collision_link_map_[joint], check_self_collision_);

        for (const auto& proxy : proxies)
        {
            bool is_robot_to_robot = (proxy.e1->is_robot_link || proxy.e1->closest_robot_link.lock()) && (proxy.e2->is_robot_link || proxy.e2->closest_robot_link.lock());
            double& margin = is_robot_to_robot ? robot_margin_ : world_margin_;

            if (proxy.distance < margin)
            {
                // Cost
                d += std::pow((1. - proxy.distance / margin), linear_ ? 1 : 2);

                if (updateJacobian)
                {
                    // Jacobian
                    KDL::Frame arel = KDL::Frame(proxy.e1->frame.Inverse(KDL::Vector(
                        proxy.contact1(0), proxy.contact1(1), proxy.contact1(2))));
                    KDL::Frame brel = KDL::Frame(proxy.e2->frame.Inverse(KDL::Vector(
                        proxy.contact2(0), proxy.contact2(1), proxy.contact2(2))));

                    if (!linear_)
                    {
                        Eigen::MatrixXd tmpJ = scene_->GetKinematicTree().Jacobian(
                            proxy.e1, arel, nullptr, KDL::Frame());
                        J += (2. / (margin * margin)) * (proxy.normal1.transpose() * tmpJ.topRows<3>());
                        tmpJ = scene_->GetKinematicTree().Jacobian(proxy.e2, brel, nullptr,
                                                                   KDL::Frame());
                        J -= (2. / (margin * margin)) * (proxy.normal1.transpose() * tmpJ.topRows<3>());
                    }
                    else
                    {
                        Eigen::MatrixXd tmpJ = scene_->GetKinematicTree().Jacobian(
                            proxy.e1, arel, nullptr, KDL::Frame());
                        J += 1 / margin * (proxy.normal1.transpose() * tmpJ.topRows<3>());
                        tmpJ = scene_->GetKinematicTree().Jacobian(proxy.e2, brel, nullptr,
                                                                   KDL::Frame());
                        J -= 1 / margin * (proxy.normal1.transpose() * tmpJ.topRows<3>());
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

    // Get names of all controlled joints and their corresponding child links
    robot_joints_ = scene_->GetControlledJointNames();
    controlled_joint_to_collision_link_map_ = scene_->GetControlledJointToCollisionLinkMap();
}

int SmoothCollisionDistance::TaskSpaceDim() { return dim_; }
}  // namespace exotica
