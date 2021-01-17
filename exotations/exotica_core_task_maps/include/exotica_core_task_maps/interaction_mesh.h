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

#ifndef EXOTICA_CORE_TASK_MAPS_INTERACTION_MESH_H_
#define EXOTICA_CORE_TASK_MAPS_INTERACTION_MESH_H_

#include <exotica_core/kinematic_tree.h>
#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/interaction_mesh_initializer.h>

#include <visualization_msgs/Marker.h>

namespace exotica
{
class InteractionMesh : public TaskMap, public Instantiable<InteractionMeshInitializer>
{
public:
    InteractionMesh();
    virtual ~InteractionMesh();

    void Instantiate(const InteractionMeshInitializer& init) override;
    void AssignScene(ScenePtr scene) override;

    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    int TaskSpaceDim() override;

    void SetWeight(int i, int j, double weight);
    void SetWeights(const Eigen::MatrixXd& weights);
    Eigen::MatrixXd GetWeights();

    static Eigen::VectorXd ComputeLaplace(Eigen::VectorXdRefConst eff_Phi, Eigen::MatrixXdRefConst weights, Eigen::MatrixXd* dist = nullptr, Eigen::VectorXd* wsum = nullptr);
    void ComputeGoalLaplace(const Eigen::VectorXd& x, Eigen::VectorXd& goal);
    static void ComputeGoalLaplace(const std::vector<KDL::Frame>& nodes, Eigen::VectorXd& goal, Eigen::MatrixXdRefConst weights);

protected:
    void Debug(Eigen::VectorXdRefConst phi);
    void InitializeDebug(std::string ref);
    void DestroyDebug();

    Eigen::MatrixXd weights_;
    int eff_size_ = 0;

    ros::Publisher imesh_mark_pub_;
    visualization_msgs::Marker imesh_mark_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_INTERACTION_MESH_H_
