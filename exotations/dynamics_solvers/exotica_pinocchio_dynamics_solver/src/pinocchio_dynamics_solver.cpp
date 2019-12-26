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
void PinocchioDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const bool verbose = false;
    if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    }
    else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::PLANAR)
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

    pinocchio_data_.reset(new pinocchio::Data(model_));
}

Eigen::VectorXd PinocchioDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    // TODO: THIS DOES NOT WORK FOR A FLOATING BASE YET!!
    pinocchio::aba(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u);
    Eigen::VectorXd x_dot(num_positions_ + num_velocities_);
    x_dot.head(num_positions_) = x.tail(num_positions_);
    x_dot.tail(num_velocities_) = pinocchio_data_->ddq;
    return x_dot;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    const int NQ = num_positions_;
    const int NV = num_velocities_;
    const int NX = NQ + NV;

    pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

    Eigen::MatrixXd fx_symb = Eigen::MatrixXd::Zero(NX, NX);
    fx_symb.topRightCorner(NV, NV) = Eigen::MatrixXd::Identity(NV, NV);
    fx_symb.bottomLeftCorner(NQ, NV) = pinocchio_data_->ddq_dq;

    return fx_symb;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    const int NQ = num_positions_;
    const int NV = num_velocities_;
    const int NX = NQ + NV;
    const int NU = num_controls_;

    pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

    Eigen::MatrixXd fu_symb = Eigen::MatrixXd::Zero(NX, NU);
    fu_symb.bottomRightCorner(NV, NU) = pinocchio_data_->Minv;

    return fu_symb;
}

Eigen::VectorXd PinocchioDynamicsSolver::InverseDynamics(const StateVector& x)
{
    // compute dynamic drift -- Coriolis, centrifugal, gravity
    // Assume 0 acceleration
    Eigen::VectorXd u = pinocchio::rnea(model_, *pinocchio_data_,
                                        x.head(num_positions_).eval(), x.tail(num_velocities_).eval(),
                                        Eigen::VectorXd::Zero(num_velocities_).eval());

    return u;
}

}  // namespace exotica
