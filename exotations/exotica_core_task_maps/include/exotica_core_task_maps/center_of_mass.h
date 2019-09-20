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

#ifndef EXOTICA_CORE_TASK_MAPS_COM_H_
#define EXOTICA_CORE_TASK_MAPS_COM_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/center_of_mass_initializer.h>

#include <visualization_msgs/MarkerArray.h>

namespace exotica
{
class CenterOfMass : public TaskMap, public Instantiable<CenterOfMassInitializer>
{
public:
    void AssignScene(ScenePtr scene) override;

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

private:
    void Initialize();
    void InitializeDebug();

    Eigen::VectorXd mass_;
    ros::Publisher com_links_pub_;
    ros::Publisher com_pub_;
    ros::Publisher goal_pub_;
    visualization_msgs::Marker com_links_marker_;
    visualization_msgs::Marker com_marker_;
    visualization_msgs::Marker goal_marker_;
    bool enable_z_;
    int dim_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_COM_H_
