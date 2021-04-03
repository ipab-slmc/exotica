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

#include <pinocchio/algorithm/cholesky.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

REGISTER_DYNAMICS_SOLVER_TYPE("PinocchioDynamicsSolverWithGravityCompensation", exotica::PinocchioDynamicsSolverWithGravityCompensation)

namespace exotica
{
void PinocchioDynamicsSolverWithGravityCompensation::ComputeDerivatives(const StateVector& x, const ControlVector& u)
{
    Eigen::VectorBlock<const Eigen::VectorXd> q = x.head(num_positions_);
    Eigen::VectorBlock<const Eigen::VectorXd> v = x.tail(num_velocities_);

    // Obtain torque to compensate gravity and dynamic effects (Coriolis)
    u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_.get(), q, v);

    // Commanded torque is u_nle_ + u
    u_command_.noalias() = u_nle_ + u;

    pinocchio_data_->Minv.setZero();
    pinocchio::computeAllTerms(model_, *pinocchio_data_.get(), q, v);
    pinocchio::cholesky::decompose(model_, *pinocchio_data_.get());
    pinocchio::cholesky::computeMinv(model_, *pinocchio_data_.get(), pinocchio_data_->Minv);

    // du_command_dq_
    a_.noalias() = pinocchio_data_->Minv * u_command_;
    pinocchio::computeRNEADerivatives(model_, *pinocchio_data_.get(), q, v, a_);
    du_command_dq_.noalias() = pinocchio_data_->Minv * pinocchio_data_->dtau_dq;

    // du_nle_dq_
    a_.noalias() = pinocchio_data_->Minv * u_nle_;
    pinocchio::computeRNEADerivatives(model_, *pinocchio_data_.get(), q, v, a_);
    du_nle_dq_.noalias() = pinocchio_data_->Minv * pinocchio_data_->dtau_dq;

    // du_dq_
    fx_.block(num_velocities_, 0, num_velocities_, num_velocities_).noalias() = du_nle_dq_ - du_command_dq_;

    // Since dtau_du=Identity, the partial derivative of fu is directly Minv.
    fu_.bottomRightCorner(num_velocities_, num_velocities_) = pinocchio_data_->Minv;

    Eigen::Block<Eigen::MatrixXd> da_dx = fx_.block(num_velocities_, 0, num_velocities_, get_num_state_derivative());
    Eigen::Block<Eigen::MatrixXd> da_du = fu_.block(num_velocities_, 0, num_velocities_, num_controls_);

    switch (integrator_)
    {
        // Forward Euler (RK1)
        case Integrator::RK1:
        {
            Fx_.topRows(num_velocities_).setZero();
            Fx_.bottomRows(num_velocities_).noalias() = dt_ * da_dx;
            Fx_.topRightCorner(num_velocities_, num_velocities_).diagonal().array() += dt_;
            pinocchio::dIntegrateTransport(model_, x.head(num_positions_), x.tail(num_velocities_), Fx_.topRows(num_velocities_), pinocchio::ARG1);
            pinocchio::dIntegrate(model_, x.head(num_positions_), x.tail(num_velocities_), Fx_.topLeftCorner(num_velocities_, num_velocities_), pinocchio::ARG0, pinocchio::ADDTO);
            Fx_.bottomRightCorner(num_velocities_, num_velocities_).diagonal().array() += 1.0;

            Fu_.topRows(num_velocities_).setZero();
            Fu_.bottomRows(num_velocities_).noalias() = dt_ * da_du;
            pinocchio::dIntegrateTransport(model_, x.head(num_positions_), x.tail(num_velocities_), Fu_.topRows(num_velocities_), pinocchio::ARG1);
        }
        break;
        // Semi-implicit Euler
        case Integrator::SymplecticEuler:
        {
            a_.noalias() = pinocchio_data_->Minv * u_command_;
            Eigen::VectorXd dx_v = dt_ * x.tail(num_velocities_) + dt_ * dt_ * a_;

            Fx_.topRows(num_velocities_).noalias() = dt_ * dt_ * da_dx;
            Fx_.bottomRows(num_velocities_).noalias() = dt_ * da_dx;
            Fx_.topRightCorner(num_velocities_, num_velocities_).diagonal().array() += dt_;
            pinocchio::dIntegrateTransport(model_, x.head(num_positions_), dx_v, Fx_.topRows(num_velocities_), pinocchio::ARG1);
            pinocchio::dIntegrate(model_, x.head(num_positions_), dx_v, Fx_.topLeftCorner(num_velocities_, num_velocities_), pinocchio::ARG0, pinocchio::ADDTO);
            Fx_.bottomRightCorner(num_velocities_, num_velocities_).diagonal().array() += 1.0;

            Fu_.topRows(num_velocities_).noalias() = dt_ * dt_ * da_du;
            Fu_.bottomRows(num_velocities_).noalias() = dt_ * da_du;
            pinocchio::dIntegrateTransport(model_, x.head(num_positions_), dx_v, Fu_.topRows(num_velocities_), pinocchio::ARG1);
        }
        break;
        default:
            ThrowPretty("Not implemented!");
    };
}

Eigen::MatrixXd PinocchioDynamicsSolverWithGravityCompensation::fx(const StateVector& x, const ControlVector& u)
{
    // This causes redundant computations but this method is no longer called in solvers.
    ComputeDerivatives(x, u);

    /*Eigen::VectorBlock<const Eigen::VectorXd> q = x.head(num_positions_);
    Eigen::VectorBlock<const Eigen::VectorXd> v = x.tail(num_velocities_);

    // Obtain torque to compensate gravity and dynamic effects (Coriolis)
    u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_.get(), q, v);

    // Commanded torque is u_nle_ + u
    u_command_.noalias() = u_nle_ + u;

    pinocchio_data_->Minv.setZero();
    pinocchio::computeAllTerms(model_, *pinocchio_data_.get(), q, v);
    pinocchio::cholesky::decompose(model_, *pinocchio_data_.get());
    pinocchio::cholesky::computeMinv(model_, *pinocchio_data_.get(), pinocchio_data_->Minv);

    // du_command_dq_
    a_.noalias() = pinocchio_data_->Minv * u_command_;
    pinocchio::computeRNEADerivatives(model_, *pinocchio_data_.get(), q, v, a_);
    du_command_dq_.noalias() = pinocchio_data_->Minv * pinocchio_data_->dtau_dq;

    // du_nle_dq_
    a_.noalias() = pinocchio_data_->Minv * u_nle_;
    pinocchio::computeRNEADerivatives(model_, *pinocchio_data_.get(), q, v, a_);
    du_nle_dq_.noalias() = pinocchio_data_->Minv * pinocchio_data_->dtau_dq;

    // du_dq_
    fx_.block(num_velocities_, 0, num_velocities_, num_velocities_).noalias() = du_nle_dq_ - du_command_dq_;*/

    return fx_;
}

Eigen::MatrixXd PinocchioDynamicsSolverWithGravityCompensation::fu(const StateVector& x, const ControlVector& u)
{
    // This causes redundant computations but this method is no longer called in solvers.
    ComputeDerivatives(x, u);

    /*Eigen::VectorBlock<const Eigen::VectorXd> q = x.head(num_positions_);
    Eigen::VectorBlock<const Eigen::VectorXd> v = x.tail(num_velocities_);

    // Obtain torque to compensate gravity and dynamic effects (Coriolis)
    u_nle_ = pinocchio::nonLinearEffects(model_, *pinocchio_data_.get(), q, v);

    // Commanded torque is u_nle_ + u
    u_command_.noalias() = u_nle_ + u;

    // Since dtau_du=Identity, the partial derivative of fu is directly Minv.
    Eigen::Block<Eigen::MatrixXd> Minv = fu_.bottomRightCorner(num_velocities_, num_velocities_);

    pinocchio::computeAllTerms(model_, *pinocchio_data_.get(), q, v);
    pinocchio::cholesky::decompose(model_, *pinocchio_data_.get());
    pinocchio::cholesky::computeMinv(model_, *pinocchio_data_.get(), Minv);*/

    return fu_;
}

}  // namespace exotica
