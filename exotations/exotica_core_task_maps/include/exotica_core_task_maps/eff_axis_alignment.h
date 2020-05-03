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

#ifndef EXOTICA_CORE_TASK_MAPS_EFF_AXIS_ALIGNMENT_H_
#define EXOTICA_CORE_TASK_MAPS_EFF_AXIS_ALIGNMENT_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/eff_axis_alignment_initializer.h>
#include <exotica_core_task_maps/frame_with_axis_and_direction_initializer.h>

#include <visualization_msgs/MarkerArray.h>

namespace exotica
{
class EffAxisAlignment : public TaskMap, public Instantiable<EffAxisAlignmentInitializer>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void AssignScene(ScenePtr scene) override;

    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;

    int TaskSpaceDim() override;

    void SetDirection(const std::string& frame_name, const Eigen::Vector3d& dir_in);
    Eigen::Vector3d GetDirection(const std::string& frame_name) const;

    void SetAxis(const std::string& frame_name, const Eigen::Vector3d& axis_in);
    Eigen::Vector3d GetAxis(const std::string& frame_name) const;

    int N;

private:
    void Initialize();

    ros::Publisher pub_debug_;
    visualization_msgs::MarkerArray msg_debug_;

    int n_frames_;

    Eigen::Matrix3Xd axis_, dir_;
    Eigen::Vector3d link_position_in_base_, link_axis_position_in_base_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_EFF_AXIS_ALIGNMENT_H_
