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
    base_parameters_ = parameters_;
}

void AnalyticDDPSolver::BackwardPass()
{
    constexpr double min_clamp_ = -1e10;
    constexpr double max_clamp_ = 1e10;
    const int T = prob_->get_T();
    const double dt = dynamics_solver_->get_dt();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const double dt_squared = dynamics_solver_->get_dt() * dynamics_solver_->get_dt();

    Eigen::MatrixXd Qx, Qu, Qxx, Quu, Qux, Quu_inv, Vxx;
    Eigen::VectorXd Vx;

    // V = prob_->GetStateCost(T - 1);
    Vx = prob_->GetStateCostJacobian(T - 1);
    Vxx = prob_->GetStateCostHessian(T - 1);

    // concatenation axis for tensor products
    //  See https://eigen.tuxfamily.org/dox-devel/unsupported/eigen_tensors.html#title14
    Eigen::array<Eigen::IndexPair<int>, 1> dims = {Eigen::IndexPair<int>(1, 0)};

    for (int t = T - 2; t >= 0; t--)
    {
        const Eigen::VectorXd x = prob_->get_X(t), u = prob_->get_U(t);
        Eigen::MatrixXd fx = dynamics_solver_->fx(x, u),
                        fu = dynamics_solver_->fu(x, u);

        fx = fx * dynamics_solver_->get_dt() + Eigen::MatrixXd::Identity(fx.rows(), fx.cols());
        fu = fu * dynamics_solver_->get_dt();

        // Q = prob_->GetStateCost(t) + prob_->GetControlCost(t) + V; // l + v
        Qx = dt * prob_->GetStateCostJacobian(t) + fx.transpose() * Vx;  // lx + fx @ Vx
        Qu = dt * prob_->GetControlCostJacobian(t) + fu.transpose() * Vx;

        if (parameters_.UseSecondOrderDynamics)
        {
            // clang-format off
            Eigen::Tensor<double, 1> Vx_tensor = Eigen::MatrixToTensor((Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&)Vx, NX);
            Qxx = dt * prob_->GetStateCostHessian(t) + fx.transpose() * Vxx * fx +
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fxx(x, u).contract(Vx_tensor, dims), NX, NX
                ) * dt;

            Quu = dt * prob_->GetControlCostHessian() + fu.transpose() * Vxx * fu +
                Eigen::TensorToMatrix(
                    (Eigen::Tensor<double, 2>)dynamics_solver_->fuu(x, u).contract(Vx_tensor, dims), NU, NU
                ) * dt;

            Qux = dt * prob_->GetStateControlCostHessian() + fu.transpose() * Vxx * fx +
                Eigen::TensorToMatrix((Eigen::Tensor<double, 2>)dynamics_solver_->fxu(x, u).contract(Vx_tensor, dims), NU, NX
                ) * dt;
            // clang-format on
        }
        else
        {
            Qxx = dt * prob_->GetStateCostHessian(t) + fx.transpose() * Vxx * fx;
            Quu = dt * prob_->GetControlCostHessian() + fu.transpose() * Vxx * fu;

            // NOTE: Qux = Qxu for all robotics systems I have seen
            //  this might need to be changed later on
            Qux = dt * prob_->GetStateControlCostHessian() + fu.transpose() * Vxx * fx;
        }

        // clang-format off
        //  Condition matrix for numerical stability.
        Quu_inv = (
            // Eigen::MatrixXd::Identity(Quu.rows(), Quu.cols()) * 1e-5 + Quu
            Eigen::MatrixXd::Identity(Quu.rows(), Quu.cols()) * lambda_ + Quu
        ).inverse();
        // clang-format on

        k_gains_[t] = -Quu_inv * Qu;
        K_gains_[t] = -Quu_inv * Qux;

        // V = Q - 0.5 * (Qu.transpose() * Quu_inv * Qu)(0);
        Vx = Qx - K_gains_[t].transpose() * Quu * k_gains_[t];
        Vxx = Qxx - K_gains_[t].transpose() * Quu * K_gains_[t];
    }
}
}  // namespace exotica
