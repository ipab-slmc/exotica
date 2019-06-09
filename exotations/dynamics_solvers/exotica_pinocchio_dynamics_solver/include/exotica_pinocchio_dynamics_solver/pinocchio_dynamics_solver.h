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

#ifndef EXOTICA_PINOCCHIO_DYNAMICS_SOLVER_PINOCCHIO_DYNAMICS_SOLVER_H_
#define EXOTICA_PINOCCHIO_DYNAMICS_SOLVER_PINOCCHIO_DYNAMICS_SOLVER_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_pinocchio_dynamics_solver/pinocchio_dynamics_solver_initializer.h>

/// TODO: remove this pragma once Pinocchio removes neutralConfiguration/
/// and fixes their deprecation warnings. (Relates to #547)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#pragma GCC diagnostic pop

namespace exotica
{
class PinocchioDynamicsSolver : public DynamicsSolver, public Instantiable<PinocchioDynamicsSolverInitializer>
{
public:
    PinocchioDynamicsSolver();

    void AssignScene(ScenePtr scene_in) override;

    StateVector f(const StateVector& x, const ControlVector& u) override;
    StateDerivative fx(const StateVector& x, const ControlVector& u) override;
    ControlDerivative fu(const StateVector& x, const ControlVector& u) override;

    Eigen::Tensor<double, 3> fxx(const StateVector& x, const ControlVector& u) override;
    Eigen::Tensor<double, 3> fuu(const StateVector& x, const ControlVector& u) override;
    Eigen::Tensor<double, 3> fxu(const StateVector& x, const ControlVector& u) override;
    
private:
    pinocchio::Model model_;
};
}  // namespace exotica

#endif  // EXOTICA_PINOCCHIO_DYNAMICS_SOLVER_PINOCCHIO_DYNAMICS_SOLVER_H_
