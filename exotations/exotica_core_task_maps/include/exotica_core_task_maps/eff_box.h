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

#ifndef EXOTICA_CORE_TASK_MAPS_EFF_BOX_H_
#define EXOTICA_CORE_TASK_MAPS_EFF_BOX_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/eff_box_initializer.h>
#include <exotica_core_task_maps/frame_with_box_limits_initializer.h>

namespace exotica
{
/// \class EffBox
///
/// \ingroup TaskMap
///
/// \brief Limits every given end-effector motion to a box in some reference frame.
class EffBox : public TaskMap, public Instantiable<EffBoxInitializer>
{
public:
    void Instantiate(const EffBoxInitializer& init) override;  // TODO: Allow user to use task map as both a cost term and inequality constraint
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

    void PublishObjectsAsMarkerArray();

    Eigen::Vector3d GetLowerLimit(const int eff_id) const;
    Eigen::Vector3d GetUpperLimit(const int eff_id) const;

private:
    Eigen::VectorXd eff_lower_;   ///< End-effector lower x, y, z limit.
    Eigen::VectorXd eff_upper_;   ///< End-effector upper x, y, z limit.
    int n_effs_;                  ///< Number of end-effectors.
    int three_times_n_effs_;      ///> Three multiplied by the number of end-effectors.
    ros::Publisher pub_markers_;  ///< publish marker for RViz
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_EFF_BOX_H_
