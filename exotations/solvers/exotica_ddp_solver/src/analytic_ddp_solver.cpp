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
    // NB: The DynamicTimeIndexedShootingProblem assumes row-major notation for derivatives
    //     The solvers follow DDP papers where we have a column-major notation => there will be transposes.
    Vx_.back() = prob_->GetStateCostJacobian(T_ - 1);
    Vxx_.back() = prob_->GetStateCostHessian(T_ - 1);

    // Regularization as introduced in Tassa's thesis, Eq. 24(a)
    if (lambda_ != 0.0)
    {
        Vxx_.back().diagonal().array() += lambda_;
    }

    // concatenation axis for tensor products
    //  See https://eigen.tuxfamily.org/dox-devel/unsupported/eigen_tensors.html#title14
    const Eigen::array<Eigen::IndexPair<int>, 1> dims = {Eigen::IndexPair<int>(1, 0)};
    Eigen::Tensor<double, 1> Vx_tensor;

    Eigen::VectorXd x(NX_), u(NU_);  // TODO: Replace
    // Quu_llt_ = Eigen::LLT<Eigen::MatrixXd>(NU_);  // TODO: Allocate outside
    for (int t = T_ - 2; t >= 0; t--)
    {
        x = prob_->get_X(t);  // (NX,1)
        u = prob_->get_U(t);  // (NU,1)

        // NB: ComputeDerivatives computes the derivatives of the state transition function which includes the selected integration scheme.
        dynamics_solver_->ComputeDerivatives(x, u);
        fx_[t].noalias() = dynamics_solver_->get_Fx();  // (NDX,NDX)
        fu_[t].noalias() = dynamics_solver_->get_Fu();  // (NDX,NU)

        //
        // NB: We use a modified cost function to compare across different
        // time horizons - the running cost is scaled by dt_
        //
        Qx_[t].noalias() = dt_ * prob_->GetStateCostJacobian(t);    // Eq. 20(a)            (1,NDX)^T => (NDX,1)
        Qx_[t].noalias() += fx_[t].transpose() * Vx_[t + 1];        //      lx + fx_ @ Vx_  (NDX,NDX)^T*(NDX,1)
        Qu_[t].noalias() = dt_ * prob_->GetControlCostJacobian(t);  // Eq. 20(b)            (1,NU)^T => (NU,1)
        Qu_[t].noalias() += fu_[t].transpose() * Vx_[t + 1];        //                      (NU,NDX)*(NDX,1) => (NU,1)

        Qxx_[t].noalias() = dt_ * prob_->GetStateCostHessian(t);         // Eq. 20(c)        (NDX,NDX)^T => (NDX,NDX)
        Qxx_[t].noalias() += fx_[t].transpose() * Vxx_[t + 1] * fx_[t];  //                  + (NDX,NDX)^T*(NDX,NDX)*(NDX,NDX)
        Quu_[t].noalias() = dt_ * prob_->GetControlCostHessian(t);       // Eq. 20(d)        (NU,NU)^T
        Quu_[t].noalias() += fu_[t].transpose() * Vxx_[t + 1] * fu_[t];  //                  + (NDX,NU)^T*(NDX,NDX)*(NDX,NU)
        // Qux_[t].noalias() = dt_ * prob_->GetStateControlCostHessian();          // Eq. 20(e)        (NU,NDX)
        // NB: This assumes that Lux is always 0.
        Qux_[t].noalias() = fu_[t].transpose() * Vxx_[t + 1] * fx_[t];  //                  + (NDX,NU)^T*(NDX,NDX) =>(NU,NDX)

        // The tensor product terms need to be added if second-order dynamics are considered.
        if (parameters_.UseSecondOrderDynamics && dynamics_solver_->get_has_second_order_derivatives())
        {
            Vx_tensor = Eigen::TensorMap<Eigen::Tensor<double, 1>>(Vx_[t + 1].data(), NDX_);

            Qxx_[t] += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxx(x, u).contract(Vx_tensor, dims), NDX_, NDX_) * dt_;

            Quu_[t] += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fuu(x, u).contract(Vx_tensor, dims), NU_, NU_) * dt_;

            Qux_[t] += Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxu(x, u).contract(Vx_tensor, dims), NU_, NDX_) * dt_;  // transpose?
        }

        // Control regularization for numerical stability
        Quu_[t].diagonal().array() += lambda_;

        // Compute gains using Cholesky decomposition
        Quu_inv_[t] = Quu_[t].llt().solve(Eigen::MatrixXd::Identity(NU_, NU_));
        k_[t].noalias() = -Quu_inv_[t] * Qu_[t];
        K_[t].noalias() = -Quu_inv_[t] * Qux_[t];

        // Using Cholesky decomposition:
        // Quu_llt_.compute(Quu_);
        // K_[t] = -Qux_;
        // Quu_llt_.solveInPlace(K_[t]);
        // k_[t] = -Qu_;
        // Quu_llt_.solveInPlace(k_[t]);

        // V = Q - 0.5 * (Qu_.transpose() * Quu_inv_ * Qu_)(0);

        // With regularisation:
        Vx_[t] = Qx_[t] + K_[t].transpose() * Quu_[t] * k_[t] + K_[t].transpose() * Qu_[t] + Qux_[t].transpose() * k_[t];     // Eq. 25(b)
        Vxx_[t] = Qxx_[t] + K_[t].transpose() * Quu_[t] * K_[t] + K_[t].transpose() * Qux_[t] + Qux_[t].transpose() * K_[t];  // Eq. 25(c)
        Vxx_[t] = 0.5 * (Vxx_[t] + Vxx_[t].transpose()).eval();                                                               // Ensure the Hessian of the value function is symmetric.

        // Regularization as introduced in Tassa's thesis, Eq. 24(a)
        if (lambda_ != 0.0)
        {
            Vxx_[t].diagonal().array() += lambda_;
        }
    }
}

}  // namespace exotica
