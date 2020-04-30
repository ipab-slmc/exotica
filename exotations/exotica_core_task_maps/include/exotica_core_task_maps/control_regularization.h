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

#ifndef EXOTICA_CORE_TASK_MAPS_CONTROL_REGULARIZATION_H_
#define EXOTICA_CORE_TASK_MAPS_CONTROL_REGULARIZATION_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/control_regularization_initializer.h>

namespace exotica
{
class ControlRegularization : public TaskMap, public Instantiable<ControlRegularizationInitializer>
{
public:
    void AssignScene(ScenePtr scene) override;

    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian) override;

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du, HessianRef ddphi_ddx, HessianRef ddphi_ddu, HessianRef ddphi_dxdu) override;

    int TaskSpaceDim() override;

    const std::vector<int>& get_joint_map() const;
    const Eigen::VectorXd& get_joint_ref() const;

private:
    void Initialize();
    int num_controlled_joints_;   ///! Number of controlled joints
    std::vector<int> joint_map_;  ///! Subset selection matrix
    Eigen::VectorXd joint_ref_;   ///! Joint reference value
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_CONTROL_REGULARIZATION_H_
