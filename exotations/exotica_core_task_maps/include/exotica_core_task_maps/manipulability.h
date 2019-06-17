//
// Copyright (c) 2018, University of Edinburgh
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

#ifndef EXOTICA_CORE_TASK_MAPS_MANIPULABILITY_H_
#define EXOTICA_CORE_TASK_MAPS_MANIPULABILITY_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/manipulability_initializer.h>

namespace exotica
{
/// \class Manipulability
///
/// \ingroup TaskMap
///
/// \brief Manipulability measure.
/// The manipulability measure for a robot at a given joint configuration indicates dexterity, that is, how isotropic the robot's motion is with respect to the task space motion. The measure is high when the manipulator is capable of equal motion in all directions and low when the manipulator is close to a singularity. This task map implements Yoshikawa's manipulability measure
/// \f[
///   m(x) = \sqrt{J(x)J(x)^T}
/// \f]
/// that is based on the shape of the velocity ellipsoid where \f$J(x)\f$ is the manipulator Jacobian matrix.. The task map is expressed by
/// \f[
///   \phi(x) := -m(x).
/// \f]
///
/// To use the task map as an inequality constraint the lower bound for \f$\phi\f$ should be set as a goal and each element should be negative.
///
/// Note that
///   - the associated value for \f$\rho\f$ <b>must</b> be negative in order to maximize the manipulability, and
///   - derivatives of \f$\Phi\f$ are computed using finite differences.
///
/// Todo
///   - Update user options, that is allow user to specify to compute based on translation, rotation, all, yoshikawa, or asada as in Peter Corkes' toolbox (cf. https://github.com/petercorke/robotics-toolbox-matlab/blob/0ca01aac26b4475094845982b9a5e80b02c024fd/%40SerialLink/maniplty.m#L27).
class Manipulability : public TaskMap, public Instantiable<ManipulabilityInitializer>
{
public:
    void Instantiate(const ManipulabilityInitializer& init) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    int TaskSpaceDim() override;

private:
    int n_end_effs_;     ///< Number of end-effectors.
    int n_rows_of_jac_;  ///< Number of rows from the top to extract from full jacobian. Is either 3 (position) or 6 (position and rotation).
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_MANIPULABILITY_H_
