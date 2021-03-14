//
// Copyright (c) 2021, University of Oxford
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

#ifndef EXOTICA_CORE_TASK_MAPS_DISTANCE_TO_LINE_H_
#define EXOTICA_CORE_TASK_MAPS_DISTANCE_TO_LINE_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/distance_to_line_2d_initializer.h>

#include <visualization_msgs/MarkerArray.h>

namespace exotica
{
/**
 * @brief Computes the signed distance between a point and a line defined by two points in 2D
 * 
 * @param P1 First point defining the line
 * @param P2 Second point defining the line
 * @param P3 Query point
 * @param d Signed distance
 */
void PointToLineDistance(const Eigen::Vector2d& P1, const Eigen::Vector2d& P2, const Eigen::Vector2d& P3, double& d)
{
    d = ((-P1.x() + P2.x()) * (P1.y() - P3.y()) - (P1.x() - P3.x()) * (-P1.y() + P2.y())) / (P2 - P1).norm();
}

/**
 * @brief Derivative of signed distance between a point and a line defined by two points in 2D
 */
void PointToLineDistanceDerivative(const Eigen::Vector2d& P1,
                                   const Eigen::Vector2d& P2,
                                   const Eigen::Vector2d& P3,
                                   const Eigen::MatrixXd& dP1_dq,
                                   const Eigen::MatrixXd& dP2_dq,
                                   const Eigen::MatrixXd& dP3_dq,
                                   Eigen::Ref<Eigen::MatrixXd>& derivative)
{
    double squared_distance_between_P1_and_P2 = (P2 - P1).squaredNorm();

    derivative = ((-P1.x() + P2.x()) * (P1.y() - P3.y()) - (P1.x() - P3.x()) * (-P1.y() + P2.y())) *
                     (-1.0 / 2.0 * (-P1.x() + P2.x()) * (-2 * dP1_dq.row(0) + 2 * dP2_dq.row(0)) -
                      1.0 / 2.0 * (-P1.y() + P2.y()) * (-2 * dP1_dq.row(1) + 2 * dP2_dq.row(1))) /
                     std::pow(squared_distance_between_P1_and_P2, 3.0 / 2.0) +
                 ((-P1.x() + P2.x()) * (dP1_dq.row(1) - dP3_dq.row(1)) + (P1.x() - P3.x()) * (dP1_dq.row(1) - dP2_dq.row(1)) +
                  (P1.y() - P2.y()) * (dP1_dq.row(0) - dP3_dq.row(0)) + (P1.y() - P3.y()) * (-dP1_dq.row(0) + dP2_dq.row(0))) /
                     std::sqrt(squared_distance_between_P1_and_P2);
}

class DistanceToLine2D : public TaskMap, public Instantiable<DistanceToLine2DInitializer>
{
public:
    void AssignScene(ScenePtr scene) final;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi) final;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) final;
    // TODO: Hessian
    int TaskSpaceDim() final;

private:
    void Initialize();
    ros::Publisher pub_debug_;
    visualization_msgs::MarkerArray debug_marker_array_msg_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_DISTANCE_TO_LINE_H_
