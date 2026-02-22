//
// Copyright (c) 2018, Wolfgang Merkt
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

#ifndef EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_H_
#define EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/joint_velocity_limit_initializer.h>

namespace exotica
{
/// \brief Joint Velocity Limit taskmap for time-indexed problems.
///        Penalisations of joint velocity limit violation within a specified percentage of the velocity limit.
class JointVelocityLimit : public TaskMap, public Instantiable<JointVelocityLimitInitializer>
{
public:
    JointVelocityLimit();
    virtual ~JointVelocityLimit();

    void AssignScene(ScenePtr scene) override;

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;

    int TaskSpaceDim() override;

private:
    void Initialize();

    double dt_ = 0.1;                                    ///< Timestep between subsequent time-steps (in s)
    Eigen::VectorXd limits_ = Eigen::VectorXd::Zero(1);  ///< Joint velocity limits (absolute, in rads/s)
    Eigen::VectorXd tau_ = Eigen::VectorXd::Zero(1);     ///< Joint velocity limits tolerance
    int N;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_JOINT_VELOCITY_LIMIT_H_
