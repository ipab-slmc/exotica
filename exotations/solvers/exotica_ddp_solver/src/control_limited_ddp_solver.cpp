//
// Copyright (c) 2019, University of Edinburgh
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

#include <exotica_core/tools/box_qp.h>
#include <exotica_core/tools/box_qp_old.h>
#include <exotica_ddp_solver/control_limited_ddp_solver.h>

REGISTER_MOTIONSOLVER_TYPE("ControlLimitedDDPSolver", exotica::ControlLimitedDDPSolver)

namespace exotica
{
void ControlLimitedDDPSolver::Instantiate(const ControlLimitedDDPSolverInitializer& init)
{
    parameters_ = init;
    base_parameters_ = AbstractDDPSolverInitializer(ControlLimitedDDPSolverInitializer(parameters_));
}

void ControlLimitedDDPSolver::BackwardPass()
{
    const Eigen::MatrixXd& control_limits = dynamics_solver_->get_control_limits();

    Vx_ = prob_->GetStateCostJacobian(T_ - 1);
    Vxx_ = prob_->GetStateCostHessian(T_ - 1);

    // concatenation axis for tensor products
    //  See https://eigen.tuxfamily.org/dox-devel/unsupported/eigen_tensors.html#title14
    Eigen::array<Eigen::IndexPair<int>, 1> dims = {Eigen::IndexPair<int>(1, 0)};

    for (int t = T_ - 2; t >= 0; t--)
    {
        Eigen::VectorXd x = prob_->get_X(t), u = prob_->get_U(t);

        dynamics_solver_->ComputeDerivatives(x, u);
        Eigen::MatrixXd fx = dynamics_solver_->get_fx() * dynamics_solver_->get_dt() + Eigen::MatrixXd::Identity(dynamics_solver_->get_num_state_derivative(), dynamics_solver_->get_num_state_derivative());
        Eigen::MatrixXd fu = dynamics_solver_->get_fu() * dynamics_solver_->get_dt();

        Qx_ = dt_ * prob_->GetStateCostJacobian(t) + fx.transpose() * Vx_;  // lx + fx @ Vx_
        Qu_ = dt_ * prob_->GetControlCostJacobian(t) + fu.transpose() * Vx_;

        // State regularization
        Vxx_.diagonal().array() += lambda_;

        if (parameters_.UseSecondOrderDynamics)
        {
            // clang-format off
            Eigen::Tensor<double, 1> Vx_tensor = Eigen::TensorMap<Eigen::Tensor<double, 1>>(Vx_.data(), NDX_);
            Qxx_ = dt_ * prob_->GetStateCostHessian(t) + fx.transpose() * Vxx_ * fx +
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fxx(x, u).contract(Vx_tensor, dims), NDX_, NDX_
                ) * dt_;

            Quu_ = dt_ * prob_->GetControlCostHessian() + fu.transpose() * Vxx_ * fu +
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fuu(x, u).contract(Vx_tensor, dims), NU_, NU_
                ) * dt_;

            Qux_ = dt_ * prob_->GetStateControlCostHessian() + fu.transpose() * Vxx_ * fx +
                Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxu(x, u).contract(Vx_tensor, dims), NU_, NDX_
                ) * dt_;
            // clang-format on
        }
        else
        {
            Qxx_ = dt_ * prob_->GetStateCostHessian(t) + fx.transpose() * Vxx_ * fx;
            Quu_ = dt_ * prob_->GetControlCostHessian() + fu.transpose() * Vxx_ * fu;

            // NOTE: Qux = Qxu for all robotics systems I have seen
            //  this might need to be changed later on
            Qux_ = dt_ * prob_->GetStateControlCostHessian() + fu.transpose() * Vxx_ * fx;
        }

        Eigen::VectorXd low_limit = control_limits.col(0) - u,
                        high_limit = control_limits.col(1) - u;

        // Quu_.diagonal().array() += lambda_;
        BoxQPSolution boxqp_sol;
        if (parameters_.UseNewBoxQP)
        {
            HIGHLIGHT("New boxqp");
            boxqp_sol = BoxQP(Quu_, Qu_, low_limit, high_limit, u, 0.1, 100, 1e-5, lambda_);
        }
        else
        {
            boxqp_sol = ExoticaBoxQP(Quu_, Qu_, low_limit, high_limit, u, 0.1, 100, 1e-5, lambda_);
        }

        Quu_inv_.setZero();
        for (unsigned int i = 0; i < boxqp_sol.free_idx.size(); ++i)
            for (unsigned int j = 0; j < boxqp_sol.free_idx.size(); ++j)
                Quu_inv_(boxqp_sol.free_idx[i], boxqp_sol.free_idx[j]) = boxqp_sol.Hff_inv(i, j);

        // Compute controls
        K_gains_[t] = -Quu_inv_ * Qux_;
        k_gains_[t] = boxqp_sol.x;

        for (unsigned int j = 0; j < boxqp_sol.clamped_idx.size(); ++j)
            K_gains_[t](boxqp_sol.clamped_idx[j]) = 0;

        Vx_ = Qx_ - K_gains_[t].transpose() * Quu_ * k_gains_[t];
        Vxx_ = Qxx_ - K_gains_[t].transpose() * Quu_ * K_gains_[t];
    }
}

}  // namespace exotica
