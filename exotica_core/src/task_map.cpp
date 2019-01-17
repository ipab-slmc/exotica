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
TaskMap::TaskMap() = default;
TaskMap::~TaskMap() = default;

void TaskMap::AssignScene(ScenePtr scene)
{
    scene_ = scene;
}

std::string TaskMap::Print(std::string prepend)
{
    std::string ret = Object::Print(prepend);
    return ret;
}

void TaskMap::InstantiateBase(const Initializer& init)
{
    Object::InstatiateObject(init);
    TaskMapInitializer MapInitializer(init);
    is_used = true;

    frames_.clear();

    for (Initializer& eff : MapInitializer.EndEffector)
    {
        FrameInitializer frame(eff);
        frames_.push_back(KinematicFrameRequest(frame.Link, GetFrame(frame.LinkOffset), frame.Base, GetFrame(frame.BaseOffset)));
    }
}

std::vector<KinematicFrameRequest> TaskMap::GetFrames()
{
    return frames_;
}

void TaskMap::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef Phi, Eigen::MatrixXdRef jacobian)
{
    if (jacobian.rows() != TaskSpaceDim() && jacobian.cols() != x.rows())
        ThrowNamed("Jacobian dimension mismatch!");

    // Set constants
    constexpr double h = 1e-6;
    constexpr double hi = 1.0/h;
    int ntask = TaskSpaceDim();
    int ndof = jacobian.cols();

    // Compute x/Phi using forward mapping (no jacobain)
    Update(x, Phi);

    // Backward finite differencing
    for (int i = 0; i < ndof; ++i) {

      // Setup for gradient estimation
      Eigen::VectorXd x_down = x;
      x_down(i) -= h;
      Eigen::VectorXd phi_down;
      phi_down.resize(ntask);

      // Set x_down as model state
      scene_->GetKinematicTree().SetModelState(x_down);

      // Compute phi_down using forward mapping (no jacobian)
      Update(x_down, phi_down);

      // Compute gradient estimate
      jacobian.col(i) = hi * (Phi - phi_down);
    }

    // Reset model state
    scene_->GetKinematicTree().SetModelState(x);

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
}
