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

#ifndef EXOTICA_CORE_TASK_MAPS_AVOID_LOOK_AT_SPHERE_H_
#define EXOTICA_CORE_TASK_MAPS_AVOID_LOOK_AT_SPHERE_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/avoid_look_at_sphere_initializer.h>

namespace exotica
{
/// \class AvoidLookAtSphere
///
/// \ingroup TaskMap
///
/// \brief Avoids pointing end-effector at a given spherical object.
///
/// This task map avoids pointing the end-effector a number of given spherical objects. AvoidLookAtSphere can be used as either a cost term or an inequality constraint. Each spherical object in the scene is defined by a position \f$o\in\mathbb{R}^3\f$ and radius \f$0<r\in\mathbb{R}\f$. The position and radii of each object must be given.
///
/// When using AvoidLookAtSphere as a cost term, the forward mapping defining the task map is expressed by
/// \f[
///    \Phi_i(x) := \begin{cases}(1 - \frac{d_i^2}{r_i^2})^2 & d_i<r_i\\0 & \text{otherwise}\end{cases}
/// \f]
/// for all \f$i=1{:}N\f$ where \f$N\f$ is the number of objects, \f$d\f$ is the orthogonal distance between the object center and the end-effector approach vector.
///
/// When using AvoidLookAtSphere as an inequality constraint, the forward mapping defining the task map is expressed by
/// \f[
///    \Phi_i(x) = r_i^2 - d_i^2
/// \f]
/// for all \f$i=1{:}N\f$.
///
/// \image html taskmap_avoid_look_at_sphere.png "AvoidLookAtSphere task map." width=500px
///
///
class AvoidLookAtSphere : public TaskMap, public Instantiable<AvoidLookAtSphereInitializer>
{
public:
    void Instantiate(const AvoidLookAtSphereInitializer& init) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    void UpdateAsCostWithoutJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);
    void UpdateAsInequalityConstraintWithoutJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);
    void UpdateAsCostWithJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian);
    void UpdateAsInequalityConstraintWithJacobian(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian);

    void PublishObjectsAsMarkerArray();

    void (AvoidLookAtSphere::*UpdateTaskMapWithoutJacobian)(Eigen::VectorXdRefConst, Eigen::VectorXdRef);
    void (AvoidLookAtSphere::*UpdateTaskMapWithJacobian)(Eigen::VectorXdRefConst, Eigen::VectorXdRef, Eigen::MatrixXdRef);
    int n_objects_;                  ///< Number of spherical objects.
    Eigen::VectorXd diameter_;       ///< The diameter of each object.
    Eigen::VectorXd radii_squared_;  ///< The square of each object radii.
    ros::Publisher pub_markers_;     ///< publish marker for RViz
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_AVOID_LOOK_AT_SPHERE_H_
