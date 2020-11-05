//
// Copyright (c) 2019, Christopher E. Mower
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

#ifndef EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_CONSTRAINT_H_
#define EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_CONSTRAINT_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/joint_velocity_limit_constraint_initializer.h>

namespace exotica
{
/// \brief Joint velocity limit task map for non time-indexed problems.
///
/// JointVelocityLimitConstraint constrains the joint velocity within a specified percentage of the velocity limit.
///
class JointVelocityLimitConstraint : public TaskMap, public Instantiable<JointVelocityLimitConstraintInitializer>
{
public:
    void AssignScene(ScenePtr scene) override;

    /// \brief Logs current joint state.
    /// SetPreviousJointState must be called after solve is called in a Python/C++ script is called
    /// to ensure the time-derivative is appropriately approximated.
    void SetPreviousJointState(Eigen::VectorXdRefConst joint_state);

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    int N_;                                  ///< Number of dofs for robot.
    int two_times_N_;                        ///< Two multiplied by the number of dofs for robot (task space dimension).
    Eigen::VectorXd current_joint_state_;    ///< Log of current joint state.
    Eigen::VectorXd joint_velocity_limits_;  ///< The joint velocity limits for the robot.
    double one_divided_by_dt_;               ///< Frequency (1/dt).
    Eigen::MatrixXd jacobian_;               ///< Task map jacobian matrix.
};

}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_CONSTRAINT_H_
