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

#include <exotica_ddp_solver/analytic_ddp_solver.h>

REGISTER_MOTIONSOLVER_TYPE("AnalyticDDPSolver", exotica::AnalyticDDPSolver)

namespace exotica
{
void AnalyticDDPSolver::Instantiate(const AnalyticDDPSolverInitializer& init)
{
    parameters_ = init;
    base_parameters_ = AbstractDDPSolverInitializer(AnalyticDDPSolverInitializer(parameters_));
}

void AnalyticDDPSolver::BackwardPass()
{
    Vx_ = prob_->GetStateCostJacobian(T_ - 1);
    Vxx_ = prob_->GetStateCostHessian(T_ - 1);

    // concatenation axis for tensor products
    //  See https://eigen.tuxfamily.org/dox-devel/unsupported/eigen_tensors.html#title14
    Eigen::array<Eigen::IndexPair<int>, 1> dims = {Eigen::IndexPair<int>(1, 0)};
    Eigen::Tensor<double, 1> Vx_tensor;

    Eigen::VectorXd x(NX_), u(NU_);               // TODO: Replace
    Quu_inv_.resize(NU_, NU_);                    // TODO: Allocate outside
    Quu_llt_ = Eigen::LLT<Eigen::MatrixXd>(NU_);  // TODO: Allocate outside
    for (int t = T_ - 2; t >= 0; t--)
    {
        x = prob_->get_X(t);
        u = prob_->get_U(t);

        fx_ = dt_ * dynamics_solver_->fx(x, u) + Eigen::MatrixXd::Identity(NX_, NX_);
        fu_ = dt_ * dynamics_solver_->fu(x, u);

        //
        // NB: We use a modified cost function to compare across different
        // time horizons - the running cost is scaled by dt_
        //
        Qx_ = dt_ * prob_->GetStateCostJacobian(t) + fx_.transpose() * Vx_;  // lx + fx_ @ Vx_
        Qu_ = dt_ * prob_->GetControlCostJacobian(t) + fu_.transpose() * Vx_;

        // State regularization
        Vxx_.diagonal().array() += lambda_;

        // NB: Qux = Qxu^T
        Qux_ = dt_ * prob_->GetStateControlCostHessian() + fu_.transpose() * Vxx_ * fx_;
        Qxx_ = dt_ * prob_->GetStateCostHessian(t) + fx_.transpose() * Vxx_ * fx_;
        Quu_ = dt_ * prob_->GetControlCostHessian() + fu_.transpose() * Vxx_ * fu_;

        if (parameters_.UseSecondOrderDynamics)
        {
            // clang-format off
            Vx_tensor = Eigen::TensorMap<Eigen::Tensor<double, 1>>(Vx_.data(), NX_);

            Qxx_ += 
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fxx(x, u).contract(Vx_tensor, dims), NX_, NX_
                ) * dt_;

            Quu_ += 
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fuu(x, u).contract(Vx_tensor, dims), NU_, NU_
                ) * dt_;

            Qux_ +=
                Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxu(x, u).contract(Vx_tensor, dims), NU_, NX_
                ) * dt_;
            // clang-format on
        }

        // Control regularization for numerical stability
        Quu_.diagonal().array() += lambda_;

        // Compute gains
        Quu_inv_ = Quu_.inverse();
        // Quu_inv_ = Quu_.llt().solve(Eigen::MatrixXd::Identity(Quu_.rows(), Quu_.cols()));
        k_gains_[t].noalias() = -Quu_inv_ * Qu_;
        K_gains_[t].noalias() = -Quu_inv_ * Qux_;

        // Using Cholesky decomposition:
        // Quu_llt_.compute(Quu_);
        // K_gains_[t] = -Qux_;
        // Quu_llt_.solveInPlace(K_gains_[t]);
        // k_gains_[t] = -Qu_;
        // Quu_llt_.solveInPlace(k_gains_[t]);

        // V = Q - 0.5 * (Qu_.transpose() * Quu_inv_ * Qu_)(0);
        // Vx_ = Qx_ - K_gains_[t].transpose() * Quu_ * k_gains_[t];
        // Vxx_ = Qxx_ - K_gains_[t].transpose() * Quu_ * K_gains_[t];

        // With regularisation:
        Vx_ = Qx_ + K_gains_[t].transpose() * Quu_ * k_gains_[t] + K_gains_[t].transpose() * Qu_ + Qux_.transpose() * k_gains_[t];
        Vxx_ = Qxx_ + K_gains_[t].transpose() * Quu_ * K_gains_[t] + K_gains_[t].transpose() * Qux_ + Qux_.transpose() * K_gains_[t];
        Vxx_ = 0.5 * (Vxx_ + Vxx_.transpose()).eval();
    }
}

}  // namespace exotica
