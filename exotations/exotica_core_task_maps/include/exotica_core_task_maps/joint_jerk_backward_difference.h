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

#ifndef EXOTICA_CORE_TASK_MAPS_JOINT_JERK_BACKWARD_DIFFERENCE_H_
#define EXOTICA_CORE_TASK_MAPS_JOINT_JERK_BACKWARD_DIFFERENCE_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/joint_jerk_backward_difference_initializer.h>

namespace exotica
{
/// \brief Time-derivative estimation by backward differencing.
/// JointJerkBackwardDifference uses backward differencing to
/// estimate the third time derivative of the joint state.
///
/// For more information see:
///   http://mathworld.wolfram.com/BackwardDifference.html
///
/// Here, x+qbd_ represents the simplified estimate of the third
/// time derivative.
///
class JointJerkBackwardDifference : public TaskMap, public Instantiable<JointJerkBackwardDifferenceInitializer>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void AssignScene(ScenePtr scene) override;

    /// \brief Logs previous joint state.
    /// SetPreviousJointState must be called after solve is called in a Python/C++ script is called
    /// to ensure the time-derivatives are appropriately approximated.
    /// Each previous joint state is pushed back by collumn and the given joint_state is placed
    /// in q_.col(0).
    /// Finally, we compute the new qbd_.
    void SetPreviousJointState(Eigen::VectorXdRefConst joint_state);

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    int N_;                                       ///< Number of dofs for robot.
    Eigen::Vector3d backward_difference_params_;  ///< Binomial cooeficient parameters.
    Eigen::MatrixXd q_;                           ///< Log of previous three joint states.
    Eigen::VectorXd qbd_;                         ///< x+qbd_ is a simplifed estimate of the third time derivative.
    Eigen::MatrixXd I_;                           ///< Identity matrix.
};
}

#endif  // EXOTICA_CORE_TASK_MAPS_JOINT_JERK_BACKWARD_DIFFERENCE_H_
