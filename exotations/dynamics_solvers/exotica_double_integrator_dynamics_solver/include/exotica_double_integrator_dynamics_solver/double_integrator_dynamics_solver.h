//
// Copyright (c) 2019, Wolfgang Merkt
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

#ifndef EXOTICA_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_H_
#define EXOTICA_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_double_integrator_dynamics_solver/double_integrator_dynamics_solver_initializer.h>

namespace exotica
{
class DoubleIntegratorDynamicsSolver : public DynamicsSolver, public Instantiable<DoubleIntegratorDynamicsSolverInitializer>
{
public:
    void AssignScene(ScenePtr scene_in) override;

    StateVector f(const StateVector& x, const ControlVector& u) override;
    void ComputeDerivatives(const StateVector& x, const ControlVector& u) override;
    StateDerivative fx(const StateVector& x, const ControlVector& u) override;
    ControlDerivative fu(const StateVector& x, const ControlVector& u) override;

private:
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Integrator last_integrator_;
};
}  // namespace exotica

#endif  // EXOTICA_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_DOUBLE_INTEGRATOR_DYNAMICS_SOLVER_H_
