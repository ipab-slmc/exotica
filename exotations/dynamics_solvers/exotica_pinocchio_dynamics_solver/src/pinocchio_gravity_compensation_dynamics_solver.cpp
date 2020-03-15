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

#include <exotica_pinocchio_dynamics_solver/pinocchio_gravity_compensation_dynamics_solver.h>

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/cholesky.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

REGISTER_DYNAMICS_SOLVER_TYPE("PinocchioDynamicsSolverWithGravityCompensation", exotica::PinocchioDynamicsSolverWithGravityCompensation)

namespace exotica
{
void PinocchioDynamicsSolverWithGravityCompensation::AssignScene(ScenePtr scene_in)
{
    constexpr bool verbose = false;
    if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    }
    else
    {
        ThrowPretty("This condition should never happen. Unknown BaseType.");
    }

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    num_controls_ = model_.nv;

    pinocchio_data_.reset(new pinocchio::Data(model_));

    // Pre-allocate data for f, fx, fu
    const int ndx = get_num_state_derivative();
    xdot_analytic_.setZero(ndx);
    fx_analytic_.setZero(ndx, ndx);
    fx_analytic_.topRightCorner(num_velocities_, num_velocities_).setIdentity();
    fu_analytic_.setZero(ndx, num_controls_);
    u_nle_.setZero(num_controls_);
    u_command_.setZero(num_controls_);
    a_zero_.setZero(num_velocities_);
}

Eigen::VectorXd PinocchioDynamicsSolverWithGravityCompensation::f(const StateVector& x, const ControlVector& u)
{
    // Obtain torque to compensate gravity and dynamic effects (Coriolis)
    u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval());

    // Commanded torque is u_nle_ + u
    u_command_.noalias() = u_nle_ + u;

    pinocchio::aba(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u_command_);
    xdot_analytic_.head(num_velocities_) = x.tail(num_velocities_);
    xdot_analytic_.tail(num_velocities_) = pinocchio_data_->ddq;
    return xdot_analytic_;
}

// Eigen::MatrixXd PinocchioDynamicsSolverWithGravityCompensation::fx(const StateVector& x, const ControlVector& u)
// {
//     Eigen::VectorBlock<const Eigen::VectorXd> q = x.head(num_positions_);
//     Eigen::VectorBlock<const Eigen::VectorXd> v = x.tail(num_velocities_);

//     // Obtain torque to compensate gravity and dynamic effects (Coriolis)
//     u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_, q, v);

//     // Commanded torque is u_nle_ + u
//     u_command_.noalias() = u_nle_ + u;

//     pinocchio_data_->Minv.setZero();
//     pinocchio::computeAllTerms(model_, *pinocchio_data_, q, v);
//     pinocchio::cholesky::decompose(model_, *pinocchio_data_);
//     pinocchio::cholesky::computeMinv(model_, *pinocchio_data_, pinocchio_data_->Minv);
//     Eigen::VectorXd a = pinocchio_data_->Minv * u_command_;

//     pinocchio::computeRNEADerivatives(model_, *pinocchio_data_, q, v, a);

//     fx_analytic_.block(num_velocities_, 0, num_velocities_, num_velocities_).noalias() = pinocchio_data_->Minv * (Eigen::MatrixXd::Zero(num_controls_, num_controls_) + pinocchio_data_->dtau_dq);
//     // fx_analytic_.block(num_velocities_, num_velocities_, num_velocities_, num_velocities_).setZero();

//     /*
//     pinocchio::computeRNEADerivatives(pinocchio_, d->pinocchio, q, v, d->xout);
//     d->dtau_dx.leftCols(nv) = d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
//     d->dtau_dx.rightCols(nv) = d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
//     d->Fx.noalias() = d->Minv * d->dtau_dx;
//     d->Fu.noalias() = d->Minv * d->multibody.actuation->dtau_du;
//     */

//     // Four quadrants should be: 0, Identity, ddq_dq, ddq_dv
//     // 0 and Identity are set during initialisation. Here, we pass references to ddq_dq, ddq_dv to the algorithm.
//     // pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u_command_, fx_analytic_.block(num_velocities_, 0, num_velocities_, num_velocities_), fx_analytic_.block(num_velocities_, num_velocities_, num_velocities_, num_velocities_), fu_analytic_.bottomRightCorner(num_velocities_, num_velocities_));

//     return fx_analytic_;
// }

Eigen::MatrixXd PinocchioDynamicsSolverWithGravityCompensation::fu(const StateVector& x, const ControlVector& u)
{
    Eigen::VectorBlock<const Eigen::VectorXd> q = x.head(num_positions_);
    Eigen::VectorBlock<const Eigen::VectorXd> v = x.tail(num_velocities_);

    // Obtain torque to compensate gravity and dynamic effects (Coriolis)
    u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_, q, v);

    // Commanded torque is u_nle_ + u
    u_command_.noalias() = u_nle_ + u;

    // Since dtau_du=Identity, the partial derivative of fu is directly Minv.
    Eigen::Block<Eigen::MatrixXd> Minv = fu_analytic_.bottomRightCorner(num_velocities_, num_velocities_);

    pinocchio::computeAllTerms(model_, *pinocchio_data_, q, v);
    pinocchio::cholesky::decompose(model_, *pinocchio_data_);
    pinocchio::cholesky::computeMinv(model_, *pinocchio_data_, Minv);

    return fu_analytic_;
}

}  // namespace exotica
