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

#ifndef EXOTICA_CORE_TASK_MAPS_GAZE_AT_CONSTRAINT_H_
#define EXOTICA_CORE_TASK_MAPS_GAZE_AT_CONSTRAINT_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/gaze_at_constraint_initializer.h>

namespace exotica
{
/// \class GazeAtConstraint
///
/// \ingroup TaskMap
///
/// \brief Keeps a given point within field of view of the end-effector.
///
/// Given a point \f$p = (x, y, z)^T\in\mathbb{R}^3\f$ defined in the end-effector frame, the robot motion is constrained such that it keeps \f$p\f$ within a virtual cone attahed to the end-effector. The task map is defined by
/// \f[
///   \Phi = \begin{bmatrix}x^2 + y^2 - \tan(\theta)^2z^2\\-z\end{bmatrix}.
/// \f]
/// where \f$\theta\f$ is the viewing angle.
///
/// \image html task_map_gaze_at_constraint.png "GazeAtConstraint task map." width=500px
///
class GazeAtConstraint : public TaskMap, public Instantiable<GazeAtConstraintInitializer>
{
public:
    void Instantiate(const GazeAtConstraintInitializer& init) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    Eigen::VectorXd tan_theta_squared_;  ///< The tangent squared of given viewing angle Theta.
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_GAZE_AT_CONSTRAINT_H_
