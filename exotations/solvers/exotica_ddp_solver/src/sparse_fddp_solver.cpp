
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

#include <exotica_ddp_solver/sparse_fddp_solver.h>

REGISTER_MOTIONSOLVER_TYPE("SparseFDDPSolver", exotica::SparseFDDPSolver)

namespace exotica
{
void SparseFDDPSolver::Instantiate(const SparseFDDPSolverInitializer& init)
{
    parameters_ = init;
    base_parameters_ = AbstractDDPSolverInitializer(FeasibilityDrivenDDPSolverInitializer(parameters_));

    clamp_to_control_limits_in_forward_pass_ = base_parameters_.ClampControlsInForwardPass;
    initial_regularization_rate_ = parameters_.RegularizationRate;

    l1_rate_ = parameters_.L1Rate;
    l2_rate_ = parameters_.L2Rate;
    huber_rate_ = parameters_.HuberRate;
}


bool SparseFDDPSolver::BackwardPassFDDP()
{
    Vxx_.back() = prob_->GetStateCostHessian(T_ - 1);
    Vx_.back() = prob_->GetStateCostJacobian(T_ - 1);

    if (!std::isnan(xreg_))
    {
        Vxx_.back().diagonal().array() += xreg_;
    }

    if (!is_feasible_)
    {
        Vx_.back().noalias() += Vxx_.back() * fs_.back();
    }

    for (int t = static_cast<int>(prob_->get_T()) - 2; t >= 0; --t)
    {
        const Eigen::MatrixXd& Vxx_p = Vxx_[t + 1];
        const Eigen::VectorXd& Vx_p = Vx_[t + 1];

        Qxx_[t] = dt_ * prob_->GetStateCostHessian(t);
        Qxu_[t] = dt_ * prob_->GetStateControlCostHessian().transpose();
        Quu_[t] = dt_ * prob_->GetControlCostHessian();
        Qx_[t] = dt_ * prob_->GetStateCostJacobian(t);
        Qu_[t] = dt_ * prob_->GetControlCostJacobian(t);

        fx_ = dt_ * dynamics_solver_->fx(xs_[t], us_[t]) + Eigen::MatrixXd::Identity(NDX_, NDX_);
        fu_ = dt_ * dynamics_solver_->fu(xs_[t], us_[t]);

        FxTVxx_p_.noalias() = fx_.transpose() * Vxx_p;
        FuTVxx_p_[t].noalias() = fu_.transpose() * Vxx_p;
        Qxx_[t].noalias() += FxTVxx_p_ * fx_;
        Qxu_[t].noalias() += FxTVxx_p_ * fu_;
        Quu_[t].noalias() += FuTVxx_p_[t] * fu_;
        Qx_[t].noalias() += fx_.transpose() * Vx_p;
        Qu_[t].noalias() += fu_.transpose() * Vx_p;

        if (!std::isnan(ureg_))
        {
            Quu_[t].diagonal().array() += ureg_;
        }
        
        for (int iu = 0; iu < NU_; ++ iu)
        {

            if (parameters_.LossType == "L1") {
            // Sparsity (L1) cost
                Qu_[t](iu) += dt_ * (1.0/(1 + std::exp(-l1_rate_(iu) * us_[t](iu)))
                    - 1.0/(1 + std::exp(l1_rate_(iu) * us_[t](iu))));

                Quu_[t](iu, iu) += dt_ * (2 * l1_rate_(iu) * std::exp(l1_rate_(iu) * us_[t](iu)) /
                    std::pow(1 + std::exp(l1_rate_(iu) * us_[t](iu)), 2));
            } else if (parameters_.LossType == "Huber") {
                Qu_[t](iu) += dt_ * (
                    us_[t](iu) / (
                        std::sqrt(
                            1.0 + us_[t](iu) * us_[t](iu) / (huber_rate_(iu) * huber_rate_(iu))
                        )
                    )
                );

                Quu_[t](iu, iu) += dt_ * (
                    std::pow(huber_rate_(iu), 4) * std::sqrt(
                        1.0 + us_[t](iu) * us_[t](iu) / (huber_rate_(iu) * huber_rate_(iu))
                    ) / (
                        std::pow(huber_rate_(iu) * huber_rate_(iu) + us_[t](iu) * us_[t](iu), 2)
                    )
                );
            }
    
        // (L2) cost
            Qu_[t](iu) += dt_ * 2 * l2_rate_(iu) * us_[t](iu);

            Quu_[t](iu, iu) += dt_ * 2 * l2_rate_(iu);

        }

        ComputeGains(t);

        Vx_[t] = Qx_[t];
        if (std::isnan(ureg_))
        {
            Vx_[t].noalias() -= K_[t].transpose() * Qu_[t];
        }
        else
        {
            Quuk_[t].noalias() = Quu_[t] * k_[t];
            Vx_[t].noalias() += K_[t].transpose() * Quuk_[t];
            Vx_[t].noalias() -= 2 * (K_[t].transpose() * Qu_[t]);
        }
        Vxx_[t] = Qxx_[t];
        Vxx_[t].noalias() -= Qxu_[t] * K_[t];
        Vxx_[t] = 0.5 * (Vxx_[t] + Vxx_[t].transpose()).eval();

        if (!std::isnan(xreg_))
        {
            Vxx_[t].diagonal().array() += xreg_;
        }

        // Compute and store the Vx gradient at end of the interval (rollout state)
        if (!is_feasible_)
        {
            Vx_[t].noalias() += Vxx_[t] * fs_[t];
        }

        if (IsNaN(Vx_[t].lpNorm<Eigen::Infinity>()))
        {
            return false;
        }
        if (IsNaN(Vxx_[t].lpNorm<Eigen::Infinity>()))
        {
            return false;
        }
    }

    l1_rate_  = (l1_rate_ * parameters_.L1IncreaseRate).cwiseMin(parameters_.MaxL1Rate);
    return true;
}

}  // namespace exotica
