/*
 *      Author: Christopher E. Mower
 *
 * Copyright (c) 2018, University of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_BACKWARD_DIFFERENCE_H_
#define EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_BACKWARD_DIFFERENCE_H_

#include <exotica/TaskMap.h>
#include <exotica_core_task_maps/JointVelocityBackwardDifferenceInitializer.h>

namespace exotica
{
/** \brief Time-derivative estimation by backward differencing.
 *
 *  JointVelocityBackwardDifference uses backward differencing to
 *  estimate the first time derivative of the joint state.
 *  
 *  For more information see:
 *    http://mathworld.wolfram.com/BackwardDifference.html
 * 
 *  Here, x+qbd_ represents the simplified estimate of the first
 *  time derivative.
 * 
 */
class JointVelocityBackwardDifference : public TaskMap, public Instantiable<JointVelocityBackwardDifferenceInitializer>
{
public:
    JointVelocityBackwardDifference();
    virtual ~JointVelocityBackwardDifference();
    void Instantiate(JointVelocityBackwardDifferenceInitializer& init) override;
    void assignScene(Scene_ptr scene) override;

    /** \brief Logs previous joint state.
     *
     *  set_previous_joint_state must be called after solve is called in a Python/C++ script is called 
     *  to ensure the time-derivatives are appropriately approximated. 
     *  The new joint_state is set to q_. Finally, we compute the new qbd_.
     * 
     */
    void set_previous_joint_state(Eigen::VectorXdRefConst joint_state);

    void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J) override;
    int taskSpaceDim() override;

private:
    JointVelocityBackwardDifferenceInitializer init_;  ///< Task map initializer.
    Scene_ptr scene_;                                  ///< Scene pointer.
    double backward_difference_params_;                ///< Binomial cooeficient parameters.
    int N_;                                            ///< Number of dofs for robot.
    Eigen::VectorXd q_;                                ///< Log of previous joint state.
    Eigen::VectorXd qbd_;                              ///< x+qbd_ is a simplifed estimate of the first time derivative.
    Eigen::MatrixXd I_;                                ///< Identity matrix.
    double dt_inv_;                                    ///< Frequency (1/dt)
};
}

#endif /* EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_BACKWARD_DIFFERENCE_H_ */
