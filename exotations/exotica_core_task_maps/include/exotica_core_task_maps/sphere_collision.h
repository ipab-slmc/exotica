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

#ifndef EXOTICA_CORE_TASK_MAPS_SPHERE_COLLISION_H_
#define EXOTICA_CORE_TASK_MAPS_SPHERE_COLLISION_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/sphere_collision_initializer.h>
#include <exotica_core_task_maps/sphere_initializer.h>

#include <visualization_msgs/Marker.h>

namespace exotica
{
class SphereCollision : public TaskMap, public Instantiable<SphereCollisionInitializer>
{
public:
    void Instantiate(const SphereCollisionInitializer& init) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    double Distance(const KDL::Frame& eff_A, const KDL::Frame& eff_B, double r_A, double r_B);
    Eigen::VectorXd Jacobian(const KDL::Frame& eff_A, const KDL::Frame& eff_B, const KDL::Jacobian& jacA, const KDL::Jacobian& jacB, double r_A, double r_B);

    std::map<std::string, std::vector<int>> groups_;
    std::vector<double> radiuses_;

    visualization_msgs::MarkerArray debug_msg_;
    ros::Publisher debug_pub_;
    double eps_;
    int dim_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_SPHERE_COLLISION_H_
