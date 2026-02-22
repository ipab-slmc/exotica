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
#include <exotica_ddp_solver/control_limited_feasibility_driven_ddp_solver.h>

REGISTER_MOTIONSOLVER_TYPE("ControlLimitedFeasibilityDrivenDDPSolver", exotica::ControlLimitedFeasibilityDrivenDDPSolver)

namespace exotica
{
void ControlLimitedFeasibilityDrivenDDPSolver::Instantiate(const ControlLimitedFeasibilityDrivenDDPSolverInitializer& init)
{
    exotica::Instantiable<exotica::ControlLimitedFeasibilityDrivenDDPSolverInitializer>::parameters_ = init;
    base_parameters_ = AbstractDDPSolverInitializer(ControlLimitedFeasibilityDrivenDDPSolverInitializer(exotica::Instantiable<exotica::ControlLimitedFeasibilityDrivenDDPSolverInitializer>::parameters_));

    clamp_to_control_limits_in_forward_pass_ = base_parameters_.ClampControlsInForwardPass;
    initial_regularization_rate_ = base_parameters_.RegularizationRate;
    th_stepinc_ = base_parameters_.ThresholdRegularizationIncrease;
    th_stepdec_ = base_parameters_.ThresholdRegularizationDecrease;

    th_stop_ = parameters_.GradientToleranceConvergenceThreshold;
    th_gradient_tolerance_ = parameters_.GradientTolerance;
    th_acceptstep_ = parameters_.DescentStepAcceptanceThreshold;
    th_acceptnegstep_ = parameters_.AscentStepAcceptanceThreshold;
}

void ControlLimitedFeasibilityDrivenDDPSolver::AllocateData()
{
    AbstractFeasibilityDrivenDDPSolver::AllocateData();

    Quu_inv_.resize(T_ - 1);
    for (int t = 0; t < T_ - 1; ++t)
    {
        Quu_inv_[t] = Eigen::MatrixXd::Zero(NU_, NU_);
    }

    du_lb_.resize(NU_);
    du_ub_.resize(NU_);
}

void ControlLimitedFeasibilityDrivenDDPSolver::ComputeGains(const int t)
{
    if (!is_feasible_)  // and assuming that we have control limits
    {
        // No control limits on this model: Use vanilla DDP
        AbstractFeasibilityDrivenDDPSolver::ComputeGains(t);
        return;
    }

    du_lb_ = control_limits_.col(0) - us_[t];
    du_ub_ = control_limits_.col(1) - us_[t];

    // TODO: Use pre-allocated BoxQP
    BoxQPSolution boxqp_sol;
    if (parameters_.UseNewBoxQP)
    {
        boxqp_sol = BoxQP(Quu_[t], Qu_[t], du_lb_, du_ub_, k_[t], 0.1, 100, 1e-5, ureg_, parameters_.BoxQPUsePolynomialLinesearch, parameters_.BoxQPUseCholeskyFactorization);
    }
    else
    {
        boxqp_sol = ExoticaBoxQP(Quu_[t], Qu_[t], du_lb_, du_ub_, k_[t], 0.1, 100, 1e-5, ureg_, parameters_.BoxQPUsePolynomialLinesearch, parameters_.BoxQPUseCholeskyFactorization);
    }

    // Compute controls
    Quu_inv_[t].setZero();
    if (boxqp_sol.free_idx.size() > 0)
    {
        for (std::size_t i = 0; i < boxqp_sol.free_idx.size(); ++i)
        {
            for (std::size_t j = 0; j < boxqp_sol.free_idx.size(); ++j)
            {
                Quu_inv_[t](boxqp_sol.free_idx[i], boxqp_sol.free_idx[j]) = boxqp_sol.Hff_inv(i, j);  // 8,8
            }
        }
    }
    K_[t].noalias() = Quu_inv_[t] * Qxu_[t].transpose();
    k_[t].noalias() = -boxqp_sol.x;

    // The box-QP clamped the gradient direction; this is important for accounting
    // the algorithm advancement (i.e. stopping criteria)
    if (boxqp_sol.clamped_idx.size() > 0)
    {
        for (std::size_t i = 0; i < boxqp_sol.clamped_idx.size(); ++i)
        {
            Qu_[t](boxqp_sol.clamped_idx[i]) = 0.;
        }
    }
}

}  // namespace exotica
