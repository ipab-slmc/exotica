
//
// Copyright (c) 2020, University of Edinburgh, University of Oxford
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
}

void SparseFDDPSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    AbstractDDPSolver::SpecifyProblem(pointer);

    // L1 Rate
    if (parameters_.L1Rate.size() == 0)
    {
        ThrowPretty("L1Rate not set.");  // TODO: set default...
    }
    else if (parameters_.L1Rate.size() == 1)
    {
        l1_rate_.setConstant(prob_->get_num_controls(), parameters_.L1Rate(0));
    }
    else if (parameters_.L1Rate.size() == prob_->get_num_controls())
    {
        l1_rate_ = parameters_.L1Rate;
    }
    else
    {
        ThrowPretty("L1Rate has wrong size: expected " << prob_->get_num_controls() << ", got " << parameters_.L1Rate.size());
    }

    // // L2 Rate
    // if (parameters_.L2Rate.size() == 0)
    // {
    //     ThrowPretty("L2Rate not set.");  // TODO: set default...
    // }
    // else if (parameters_.L2Rate.size() == 1)
    // {
    //     l2_rate_.setConstant(prob_->get_num_controls(), parameters_.L2Rate(0));
    // }
    // else if (parameters_.L2Rate.size() == prob_->get_num_controls())
    // {
    //     l2_rate_ = parameters_.L2Rate;
    // }
    // else
    // {
    //     ThrowPretty("L2Rate has wrong size: expected " << prob_->get_num_controls() << ", got " << parameters_.L2Rate.size());
    // }

    // Huber Rate
    if (parameters_.HuberRate.size() == 0)
    {
        ThrowPretty("HuberRate not set.");  // TODO: set default...
    }
    else if (parameters_.HuberRate.size() == 1)
    {
        huber_rate_.setConstant(prob_->get_num_controls(), parameters_.HuberRate(0));
    }
    else if (parameters_.HuberRate.size() == prob_->get_num_controls())
    {
        huber_rate_ = parameters_.HuberRate;
    }
    else
    {
        ThrowPretty("HuberRate has wrong size: expected " << prob_->get_num_controls() << ", got " << parameters_.HuberRate.size());
    }
}

double SparseFDDPSolver::GetControlCost(int t) const
{
    double cost = prob_->GetControlCost(t);

    if (parameters_.LossType == "SmoothL1")
    {
        // cost = 0.0;  // If activating this line, please also uncomment the Qu_[t].setZeros(); in the backward-pass
        const Eigen::VectorXd& u = prob_->get_U(t);
        for (int iu = 0; iu < NU_; ++iu)
        {
            cost += 1.0 / l1_rate_(iu) * (std::log(1.0 + std::exp(-l1_rate_(iu) * u(iu))) + std::log(1.0 + std::exp(l1_rate_(iu) * u(iu))));
        }
        if (!std::isfinite(cost))
        {
            cost = 0.0;  // Likely "inf" as u is too small.
            // HIGHLIGHT(t << ": " << cost << "u: " << u.transpose())
        }
        return cost;
    }
    else if (parameters_.LossType == "Huber")
    {
        const Eigen::VectorXd& u = prob_->get_U(t);
        for (int iu = 0; iu < NU_; ++iu)
        {
            cost += huber_rate_(iu) * huber_rate_(iu) * (
                std::sqrt(
                    1 + std::pow(u(iu) / huber_rate_(iu), 2)
                ) - 1.0
            );
        }
        if (!std::isfinite(cost))
        {
            cost = 0.0;  // Likely "inf" as u is too small.
            // HIGHLIGHT(t << ": " << cost << "u: " << u.transpose())
        }
        return cost;
    }
    
    return cost;
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

        // If we are in L2 mode, it's vanilla DDP and no change necessary.
        // If a different loss type is specified, adjust here:
        if (parameters_.LossType == "SmoothL1" || parameters_.LossType == "Huber")
        {
            // Qu_[t].setZero();
            // Quu_[t].setZero();

            for (int iu = 0; iu < NU_; ++iu)
            {
                if (parameters_.LossType == "SmoothL1")
                {
                    // Sparsity (L1) cost
                    Qu_[t](iu) = dt_ * (1.0 / (1 + std::exp(-l1_rate_(iu) * us_[t](iu))) - 1.0 / (1 + std::exp(l1_rate_(iu) * us_[t](iu))));

                    Quu_[t](iu, iu) = dt_ * (2 * l1_rate_(iu) * std::exp(l1_rate_(iu) * us_[t](iu)) / std::pow(1 + std::exp(l1_rate_(iu) * us_[t](iu)), 2));
                }
                else if (parameters_.LossType == "Huber")
                {
                    Qu_[t](iu) = dt_ * (us_[t](iu) / (std::sqrt(1.0 + us_[t](iu) * us_[t](iu) / (huber_rate_(iu) * huber_rate_(iu)))));

                    Quu_[t](iu, iu) = dt_ * (std::pow(huber_rate_(iu), 4) * std::sqrt(1.0 + us_[t](iu) * us_[t](iu) / (huber_rate_(iu) * huber_rate_(iu))) / (std::pow(huber_rate_(iu) * huber_rate_(iu) + us_[t](iu) * us_[t](iu), 2)));
                }
            }
        }

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

    return true;
}

void SparseFDDPSolver::OnIterationEnd()
{
    l1_rate_ = (l1_rate_ * parameters_.L1IncreaseRate).cwiseMin(parameters_.MaxL1Rate);
};

}  // namespace exotica
