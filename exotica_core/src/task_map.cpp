//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_core/task_map.h>

#include <exotica_core/frame_initializer.h>
#include <exotica_core/task_map_initializer.h>

namespace exotica
{
void TaskMap::AssignScene(ScenePtr scene)
{
    scene_ = scene;
}

void TaskMap::InstantiateBase(const Initializer& init)
{
    Object::InstantiateObject(init);
    TaskMapInitializer MapInitializer(init);
    is_used = true;

    frames_.clear();

    for (Initializer& eff : MapInitializer.EndEffector)
    {
        FrameInitializer frame(eff);
        frames_.push_back(KinematicFrameRequest(frame.Link, GetFrame(frame.LinkOffset), frame.Base, GetFrame(frame.BaseOffset)));
    }
}

std::vector<KinematicFrameRequest> TaskMap::GetFrames() const
{
    return frames_;
}

void TaskMap::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (jacobian.rows() != TaskSpaceDim() && jacobian.cols() != q.rows())
        ThrowNamed("Jacobian dimension mismatch!");

    if (scene_ == nullptr)
    {
        ThrowNamed("Scene is not initialised!");
    }

    // Set constants
    constexpr double h = 1e-6;
    constexpr double h_inverse = 1.0 / h;

    // Compute x/phi using forward mapping (no jacobian)
    Update(q, phi);

    // Setup for gradient estimate
    Eigen::VectorXd q_backward(q.size()), phi_backward(TaskSpaceDim());

    // Backward finite differencing
    for (int i = 0; i < jacobian.cols(); ++i)
    {
        // Compute and set x_backward as model state
        q_backward = q;
        q_backward(i) -= h;
        scene_->GetKinematicTree().Update(q_backward);

        // Compute phi_backward using forward mapping (no jacobian)
        Update(q_backward, phi_backward);

        // Compute gradient estimate
        jacobian.col(i).noalias() = h_inverse * (phi - phi_backward);
    }

    // Reset model state
    scene_->GetKinematicTree().Update(q);
}

void TaskMap::Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    Update(q, phi, jacobian);
    const int ndq = scene_->get_has_quaternion_floating_base() ? scene_->get_num_positions() - 1 : scene_->get_num_positions();
    for (int i = 0; i < TaskSpaceJacobianDim(); ++i)
    {
        Eigen::Block<Eigen::Ref<Eigen::MatrixXd>> jacobian_row = jacobian.block(i, 0, 1, ndq);
        hessian(i).topLeftCorner(ndq, ndq).noalias() = jacobian_row.transpose() * jacobian_row;
    }
}

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi)
{
    // WARNING("x,u update not implemented - defaulting to q update.");
    Update(x.head(scene_->get_num_positions()), phi);
}

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du)
{
    // WARNING("x,u update not implemented - defaulting to q update.");
    const int ndq = scene_->get_has_quaternion_floating_base() ? scene_->get_num_positions() - 1 : scene_->get_num_positions();
    Update(x.head(ndq), phi, dphi_dx.topLeftCorner(TaskSpaceJacobianDim(), ndq));
}

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du, HessianRef ddphi_ddx, HessianRef ddphi_ddu, HessianRef ddphi_dxdu)
{
    // WARNING("x,u update not implemented - defaulting to q update.");
    const int ndq = scene_->get_has_quaternion_floating_base() ? scene_->get_num_positions() - 1 : scene_->get_num_positions();
    Update(x.head(ndq), phi, dphi_dx.topLeftCorner(TaskSpaceJacobianDim(), ndq), ddphi_ddx);
}

}  // namespace exotica
