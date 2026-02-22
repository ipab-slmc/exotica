//
// Copyright (c) 2019-2020, University of Edinburgh, University of Oxford
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

    Vx_.back().noalias() = prob_->GetStateCostJacobian(T_ - 1);
    Vxx_.back().noalias() = prob_->GetStateCostHessian(T_ - 1);

    // concatenation axis for tensor products
    //  See https://eigen.tuxfamily.org/dox-devel/unsupported/eigen_tensors.html#title14
    Eigen::array<Eigen::IndexPair<int>, 1> dims = {Eigen::IndexPair<int>(1, 0)};

    Eigen::VectorXd x(NX_), u(NU_);  // TODO: Replace
    for (int t = T_ - 2; t >= 0; t--)
    {
        x = prob_->get_X(t);
        u = prob_->get_U(t);

        dynamics_solver_->ComputeDerivatives(x, u);
        fx_[t].noalias() = dynamics_solver_->get_Fx();
        fu_[t].noalias() = dynamics_solver_->get_Fu();

        Qx_[t].noalias() = dt_ * prob_->GetStateCostJacobian(t) + fx_[t].transpose() * Vx_[t + 1];
        Qu_[t].noalias() = dt_ * prob_->GetControlCostJacobian(t) + fu_[t].transpose() * Vx_[t + 1];

        // State regularization
        Vxx_[t + 1].diagonal().array() += lambda_;

        Qxx_[t].noalias() = dt_ * prob_->GetStateCostHessian(t) + fx_[t].transpose() * Vxx_[t + 1] * fx_[t];
        Quu_[t].noalias() = dt_ * prob_->GetControlCostHessian(t) + fu_[t].transpose() * Vxx_[t + 1] * fu_[t];
        // Qux_[t].noalias() = dt_ * prob_->GetStateControlCostHessian()  // TODO: Reactivate once we have costs that depend on both x and u!
        Qux_[t].noalias() = fu_[t].transpose() * Vxx_[t + 1] * fx_[t];

        if (parameters_.UseSecondOrderDynamics && dynamics_solver_->get_has_second_order_derivatives())
        {
            Eigen::Tensor<double, 1> Vx_tensor = Eigen::TensorMap<Eigen::Tensor<double, 1>>(Vx_[t + 1].data(), NDX_);
            Qxx_[t].noalias() += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxx(x, u).contract(Vx_tensor, dims), NDX_, NDX_) * dt_;
            Quu_[t].noalias() += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fuu(x, u).contract(Vx_tensor, dims), NU_, NU_) * dt_;
            Qux_[t].noalias() += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxu(x, u).contract(Vx_tensor, dims), NU_, NDX_) * dt_;
        }

        Eigen::VectorXd low_limit = control_limits.col(0) - u,
                        high_limit = control_limits.col(1) - u;

        // Quu_.diagonal().array() += lambda_;
        BoxQPSolution boxqp_sol;
        if (parameters_.UseNewBoxQP)
        {
            boxqp_sol = BoxQP(Quu_[t], Qu_[t], low_limit, high_limit, u, 0.1, 100, 1e-5, lambda_, parameters_.BoxQPUsePolynomialLinesearch, parameters_.BoxQPUseCholeskyFactorization);
        }
        else
        {
            boxqp_sol = ExoticaBoxQP(Quu_[t], Qu_[t], low_limit, high_limit, u, 0.1, 100, 1e-5, lambda_, parameters_.BoxQPUsePolynomialLinesearch, parameters_.BoxQPUseCholeskyFactorization);
        }

        Quu_inv_[t].setZero();
        if (boxqp_sol.free_idx.size() > 0)
            for (std::size_t i = 0; i < boxqp_sol.free_idx.size(); ++i)
                for (std::size_t j = 0; j < boxqp_sol.free_idx.size(); ++j)
                    Quu_inv_[t](boxqp_sol.free_idx[i], boxqp_sol.free_idx[j]) = boxqp_sol.Hff_inv(i, j);

        // Compute controls
        K_[t].noalias() = -Quu_inv_[t] * Qux_[t];
        k_[t].noalias() = boxqp_sol.x;

        // Update the value function w.r.t. u as k (feed-forward term) is clamped inside the BoxQP
        if (boxqp_sol.free_idx.size() > 0)
            for (std::size_t i = 0; i < boxqp_sol.clamped_idx.size(); ++i)
                Qu_[t](boxqp_sol.clamped_idx[i]) = 0.;

        Vx_[t].noalias() = Qx_[t] + K_[t].transpose() * Quu_[t] * k_[t] + K_[t].transpose() * Qu_[t] + Qux_[t].transpose() * k_[t];     // Eq. 25(b)
        Vxx_[t].noalias() = Qxx_[t] + K_[t].transpose() * Quu_[t] * K_[t] + K_[t].transpose() * Qux_[t] + Qux_[t].transpose() * K_[t];  // Eq. 25(c)
        Vxx_[t] = 0.5 * (Vxx_[t] + Vxx_[t].transpose()).eval();                                                                         // Ensure the Hessian of the value function is symmetric.
    }
}

}  // namespace exotica
