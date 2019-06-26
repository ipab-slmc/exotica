//
// Copyright (c) 2018, University of Edinburgh
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

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef Phi, Eigen::MatrixXdRef jacobian)
{
    if (jacobian.rows() != TaskSpaceDim() && jacobian.cols() != x.rows())
        ThrowNamed("Jacobian dimension mismatch!");

    if (scene_ == nullptr)
    {
        ThrowNamed("Scene is not initialised!");
    }

    // Set constants
    constexpr double h = 1e-6;
    constexpr double h_inverse = 1.0 / h;

    // Compute x/Phi using forward mapping (no jacobian)
    Update(x, Phi);

    // Setup for gradient estimate
    Eigen::VectorXd x_backward(x.size()), Phi_backward(TaskSpaceDim());

    // Backward finite differencing
    for (int i = 0; i < jacobian.cols(); ++i)
    {
        // Compute and set x_backward as model state
        x_backward = x;
        x_backward(i) -= h;
        scene_->GetKinematicTree().Update(x_backward);

        // Compute phi_backward using forward mapping (no jacobian)
        Update(x_backward, Phi_backward);

        // Compute gradient estimate
        jacobian.col(i) = h_inverse * (Phi - Phi_backward);
    }

    // Reset model state
    scene_->GetKinematicTree().Update(x);
}

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef Phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    Update(x, Phi, jacobian);
    hessian.resize(TaskSpaceDim());
    for (int i = 0; i < TaskSpaceDim(); ++i)
    {
        hessian(i) = jacobian.row(i).transpose() * jacobian.row(i);
    }
}
}  // namespace
