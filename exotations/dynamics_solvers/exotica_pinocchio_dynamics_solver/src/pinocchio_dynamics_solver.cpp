//
// Copyright (c) 2019, Wolfgang Merkt
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

#include <exotica_pinocchio_dynamics_solver/pinocchio_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("PinocchioDynamicsSolver", exotica::PinocchioDynamicsSolver)

namespace exotica
{
PinocchioDynamicsSolver::PinocchioDynamicsSolver() {}
void PinocchioDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const bool verbose = false;
    if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    }
    else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelPlanar(), model_, verbose);
    }
    else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FLOATING)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelFreeFlyer(), model_, verbose);
    }
    else
    {
        ThrowPretty("This condition should never happen. Unknown BaseType.");
    }

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    num_controls_ = model_.nv;
}

Eigen::VectorXd PinocchioDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    // TODO: THIS DOES NOT WORK FOR A FLOATING BASE YET!!
    pinocchio::Data data(model_);
    pinocchio::aba(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u);
    Eigen::VectorXd x_dot(num_positions_ + num_velocities_);
    x_dot.head(num_positions_) = x.tail(num_positions_);
    x_dot.tail(num_velocities_) = data.ddq;
    return x_dot;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    // return A_;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    // return B_;
}

Eigen::Tensor<double, 3> PinocchioDynamicsSolver::fxx(const StateVector& x, const ControlVector& u)
{
    Eigen::Tensor<double, 3> fxx(num_positions_ + num_velocities_, num_positions_ + num_velocities_, num_positions_ + num_velocities_);
    fxx.setZero();
    return fxx;
}

Eigen::Tensor<double, 3> PinocchioDynamicsSolver::fuu(const StateVector& x, const ControlVector& u)
{
    Eigen::Tensor<double, 3> fuu(num_controls_, num_positions_ + num_velocities_, num_controls_);
    fuu.setZero();
    return fuu;
}

Eigen::Tensor<double, 3> PinocchioDynamicsSolver::fxu(const StateVector& x, const ControlVector& u)
{
    Eigen::Tensor<double, 3> fxu(num_controls_, num_positions_ + num_velocities_, num_positions_ + num_velocities_);
    fxu.setZero();
    return fxu;
}

}  // namespace exotica
