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

#ifndef EXOTICA_CORE_TASK_MAPS_AVOIDLOOKAT_H_
#define EXOTICA_CORE_TASK_MAPS_AVOIDLOOKAT_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/avoid_look_at_initializer.h>

namespace exotica
{
/// \class AvoidLookAt
///
/// \ingroup TaskMap
///
/// \brief Avoids pointing end-effector at a given spherical object.
///
/// This task map avoids pointing the end-effector a number of given spherical objects. AvoidLookAt should be used as an inequality constraint only. Each spherical object in the scene is defined by a position \f$o\in\mathbb{R}^3\f$ and radius \f$0<r\in\mathbb{R}\f$. The forward mapping defining the task map is expressed by
/// \f[
///    \Phi(x) := \frac{1}{2}(r^2 - \delta^2) 
/// \f]
/// where \f$\delta\f$ is the orthogonal distance between the object center and the line defined by \f$c\in\mathbb{R}^3\f$ in the end-effector frame.
///
/// \image html avoid_look_at.png "AvoidLookAt task map." width=500px
///
/// The task map relies on defining a custom frame in the end-effector frame called EffPoint (this must be the first frame defined in the XML configuration file) and a frame for each object preceeding EffPoint. The radii of each object must be given also. 
///
///  
class AvoidLookAt : public TaskMap, public Instantiable<AvoidLookAtInitializer>
{
public:
    AvoidLookAt();
    virtual ~AvoidLookAt();

    void Instantiate(AvoidLookAtInitializer& init) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
  int n_objects_;  ///< Number of spherical objects.
  Eigen::VectorXd radii2_; ///< The square of each object radii. 
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_AVOIDLOOKAT_H_
