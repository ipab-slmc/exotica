//
// Copyright (c) 2020, University of Oxford
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

#include "exotica_core_task_maps/variable_size_collision_distance.h"

REGISTER_TASKMAP_TYPE("VariableSizeCollisionDistance", exotica::VariableSizeCollisionDistance);

namespace exotica
{
void VariableSizeCollisionDistance::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != static_cast<int>(dim_)) ThrowNamed("Wrong size of phi!");
    Eigen::MatrixXd J;
    UpdateInternal(x, phi, J, false);
}

void VariableSizeCollisionDistance::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != static_cast<int>(dim_)) ThrowNamed("Wrong size of phi!");
    if (J.rows() != static_cast<int>(dim_) || J.cols() != scene_->GetKinematicTree().GetNumControlledJoints()) ThrowNamed("Wrong size of Jacobian!");
    UpdateInternal(x, phi, J, true);
}

void VariableSizeCollisionDistance::UpdateInternal(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J, bool updateJacobian)
{
    std::vector<CollisionProxy> proxies = cscene_->GetRobotToWorldCollisionDistance(world_margin_);

    // Figure out if dim_ or size of proxies is larger:
    if (proxies.size() > dim_) WARNING("Too many proxies!");
    int max_dim = std::min(proxies.size(), dim_);

    KDL::Frame arel, brel;
    Eigen::MatrixXd J_a(6, scene_->GetKinematicTree().GetNumControlledJoints()), J_b(6, scene_->GetKinematicTree().GetNumControlledJoints());
    for (int i = 0; i < max_dim; ++i)
    {
        const CollisionProxy& proxy = proxies[i];

        if (proxy.distance > world_margin_)
        {
            // Zero. Do nothing.
        }
        else
        {
            phi(i) = world_margin_ - proxy.distance;

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

                J.row(i) += proxy.normal1.transpose() * J_a.topRows<3>();
                J.row(i) -= proxy.normal1.transpose() * J_b.topRows<3>();
            }
        }
    }
}

void VariableSizeCollisionDistance::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void VariableSizeCollisionDistance::Initialize()
{
    cscene_ = scene_->GetCollisionScene();
    world_margin_ = parameters_.WorldMargin;

    dim_ = static_cast<int>(parameters_.Dimension);
    if (dim_ <= 0) ThrowNamed("Dimension needs to be greater than equal to 1, given: " << dim_);

    if (debug_)
    {
        HIGHLIGHT_NAMED("Variable Size Collision Distance", "Dimension: " << dim_ << " - World Margin: " << world_margin_);
    }
}

int VariableSizeCollisionDistance::TaskSpaceDim() { return dim_; }
}  // namespace exotica
