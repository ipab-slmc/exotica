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

#ifndef EXOTICA_DDP_SOLVER_ANALYTIC_DDP_SOLVER_H_
#define EXOTICA_DDP_SOLVER_ANALYTIC_DDP_SOLVER_H_

#include <exotica_ddp_solver/abstract_ddp_solver.h>
#include <exotica_ddp_solver/analytic_ddp_solver_initializer.h>
#include <unsupported/Eigen/CXX11/Tensor>

namespace exotica
{
// \brief DDP solver using the analytic expansion of the value-function Q.
//  Mayne, D. Q. (1966). "A second-order gradient method of optimizing non-linear discrete time systems".
class AnalyticDDPSolver : public AbstractDDPSolver, public Instantiable<AnalyticDDPSolverInitializer>
{
public:
    void Instantiate(const AnalyticDDPSolverInitializer& init) override;

private:
    ///\brief Computes the control gains for a the trajectory in the associated
    ///     DynamicTimeIndexedProblem.
    void BackwardPass() override;

    Eigen::LLT<Eigen::MatrixXd> Quu_llt_;
};
}  // namespace exotica

#endif  // EXOTICA_DDP_SOLVER_ANALYTIC_DDP_SOLVER_H_
