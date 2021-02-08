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

#include "exotica_core_task_maps/collision_distance.h"

REGISTER_TASKMAP_TYPE("CollisionDistance", exotica::CollisionDistance);

namespace exotica
{
void CollisionDistance::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    Eigen::MatrixXd J(dim_, dim_);
    UpdateInternal(x, phi, J, false);
}

void CollisionDistance::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    phi.setZero();
    J.setZero();
    UpdateInternal(x, phi, J, true);
}

void CollisionDistance::UpdateInternal(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J, bool updateJacobian)
{
    if (!scene_->AlwaysUpdatesCollisionScene())
        cscene_->UpdateCollisionObjectTransforms();

    // For all robot links: Get all collision distances, sort by distance, and process the closest.
    for (int i = 0; i < dim_; ++i)
    {
        std::vector<CollisionProxy> proxies = cscene_->GetCollisionDistance(controlled_joint_to_collision_link_map_[robot_joints_[i]], check_self_collision_);
        if (proxies.size() == 0)
        {
            // phi(i) = 0;
            // J.row(i).setZero();
            continue;
        }

        CollisionProxy& closest_proxy = closest_proxies_[i];
        closest_proxy.distance = std::numeric_limits<double>::max();
        for (const auto& tmp_proxy : proxies)
        {
            const bool is_robot_to_robot = (tmp_proxy.e1->is_robot_link || tmp_proxy.e1->closest_robot_link.lock()) && (tmp_proxy.e2->is_robot_link || tmp_proxy.e2->closest_robot_link.lock());
            const double& margin = is_robot_to_robot ? robot_margin_ : world_margin_;
            if ((tmp_proxy.distance - margin) < closest_proxy.distance)
            {
                closest_proxy = tmp_proxy;
                closest_proxy.distance -= margin;
            }
        }

        phi(i) = closest_proxy.distance;

        if (updateJacobian)
        {
            KDL::Frame arel = KDL::Frame(closest_proxy.e1->frame.Inverse(KDL::Vector(
                closest_proxy.contact1(0), closest_proxy.contact1(1), closest_proxy.contact1(2))));
            KDL::Frame brel = KDL::Frame(closest_proxy.e2->frame.Inverse(KDL::Vector(
                closest_proxy.contact2(0), closest_proxy.contact2(1), closest_proxy.contact2(2))));

            Eigen::MatrixXd tmpJ = scene_->GetKinematicTree().Jacobian(
                closest_proxy.e1, arel, nullptr, KDL::Frame());
            J.row(i) += (closest_proxy.normal1.transpose() * tmpJ.topRows<3>());
            tmpJ = scene_->GetKinematicTree().Jacobian(closest_proxy.e2, brel, nullptr,
                                                       KDL::Frame());
            J.row(i) -= (closest_proxy.normal1.transpose() * tmpJ.topRows<3>());
        }
    }

    J *= -1;  // yup, this is was wrong since forever.
}

void CollisionDistance::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void CollisionDistance::Initialize()
{
    cscene_ = scene_->GetCollisionScene();
    check_self_collision_ = parameters_.CheckSelfCollision;
    world_margin_ = parameters_.WorldMargin;
    robot_margin_ = parameters_.RobotMargin;

    // Get names of all controlled joints and their corresponding child links
    robot_joints_ = scene_->GetControlledJointNames();
    controlled_joint_to_collision_link_map_ = scene_->GetControlledJointToCollisionLinkMap();
    dim_ = static_cast<int>(robot_joints_.size());
    closest_proxies_.assign(dim_, CollisionProxy());
    if (debug_)
    {
        HIGHLIGHT_NAMED("Collision Distance", "Dimension: " << dim_
                                                            << " - CheckSelfCollision: " << check_self_collision_
                                                            << "World Margin: " << world_margin_
                                                            << " Robot Margin: " << robot_margin_);
    }
}

int CollisionDistance::TaskSpaceDim() { return dim_; }
}  // namespace exotica
