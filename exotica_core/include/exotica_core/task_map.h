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

#ifndef EXOTICA_CORE_TASK_MAP_H_
#define EXOTICA_CORE_TASK_MAP_H_

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>  // Generally dense manipulations should be enough

#include <exotica_core/factory.h>  // The Factory template
#include <exotica_core/object.h>   // The EXOTica base class
#include <exotica_core/property.h>
#include <exotica_core/scene.h>
#include <exotica_core/task_space_vector.h>

///
/// \brief Convenience registrar for the TaskMap Type
///
#define REGISTER_TASKMAP_TYPE(TYPE, DERIV) EXOTICA_CORE_REGISTER(exotica::TaskMap, TYPE, DERIV)

namespace exotica
{
class TaskMap : public Object, Uncopyable, public virtual InstantiableBase
{
public:
    virtual void InstantiateBase(const Initializer& init);

    virtual void AssignScene(ScenePtr scene);

    // Kinematic-only task maps (traditional) use the following update methods:
    // x renamed to q in declaration: configuration only
    virtual void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi) = 0;
    virtual void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian);
    virtual void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian);

    //------------- New TaskMap API -------------
    // x => full state of q,v
    // u => controls
    virtual void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi);
    virtual void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du);
    virtual void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef dphi_dx, Eigen::MatrixXdRef dphi_du, HessianRef ddphi_ddx, HessianRef ddphi_ddu, HessianRef ddphi_dxdu);

    virtual int TaskSpaceDim() = 0;
    virtual int TaskSpaceJacobianDim() { return TaskSpaceDim(); }
    virtual void PreUpdate() {}
    virtual std::vector<TaskVectorEntry> GetLieGroupIndices() { return std::vector<TaskVectorEntry>(); }
    std::vector<KinematicFrameRequest> GetFrames() const;

    std::vector<KinematicSolution> kinematics = std::vector<KinematicSolution>(1);
    int id = -1;
    int start = -1;
    int length = -1;
    int start_jacobian = -1;
    int length_jacobian = -1;
    bool is_used = false;

protected:
    std::vector<KinematicFrameRequest> frames_;
    ScenePtr scene_ = nullptr;
};

// Typedefines for some common functionality
typedef std::shared_ptr<TaskMap> TaskMapPtr;           //!< Task Map smart pointer
typedef std::map<std::string, TaskMapPtr> TaskMapMap;  //!< The mapping by name of TaskMaps
typedef std::vector<TaskMapPtr> TaskMapVec;
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAP_H_
