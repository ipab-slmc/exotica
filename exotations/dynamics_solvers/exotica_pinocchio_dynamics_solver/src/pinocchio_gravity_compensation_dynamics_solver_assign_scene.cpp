//
// Copyright (c) 2020, University of Oxford
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

#include <exotica_pinocchio_dynamics_solver/pinocchio_gravity_compensation_dynamics_solver.h>

#include <pinocchio/parsers/urdf.hpp>

namespace exotica
{
void PinocchioDynamicsSolverWithGravityCompensation::AssignScene(ScenePtr scene_in)
{
    constexpr bool verbose = false;
    if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    }
    else
    {
        ThrowPretty("Only BaseType::FIXED is currently supported with this DynamicsSolver.");
    }

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    num_controls_ = model_.nv;

    pinocchio_data_.reset(new pinocchio::Data(model_));

    // Pre-allocate data for f, fx, fu
    const int ndx = get_num_state_derivative();
    xdot_analytic_.setZero(ndx);
    fx_.setZero(ndx, ndx);
    fx_.topRightCorner(num_velocities_, num_velocities_).setIdentity();
    fu_.setZero(ndx, num_controls_);
    Fx_.setZero(ndx, ndx);
    Fu_.setZero(ndx, num_controls_);
    u_nle_.setZero(num_controls_);
    u_command_.setZero(num_controls_);
    a_.setZero(num_velocities_);
    du_command_dq_.setZero(num_controls_, num_velocities_);
    du_nle_dq_.setZero(num_controls_, num_velocities_);
}
}  // namespace exotica
