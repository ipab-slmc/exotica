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

#ifndef EXOTICA_CORE_BOUNDED_END_POSE_PROBLEM_H_
#define EXOTICA_CORE_BOUNDED_END_POSE_PROBLEM_H_

#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/bounded_end_pose_problem_initializer.h>

namespace exotica
{
/// \brief Bound constrained end-pose problem implementation
class BoundedEndPoseProblem : public PlanningProblem, public Instantiable<BoundedEndPoseProblemInitializer>
{
public:
    BoundedEndPoseProblem();
    virtual ~BoundedEndPoseProblem();

    virtual void Instantiate(const BoundedEndPoseProblemInitializer& init);
    void Update(Eigen::VectorXdRefConst x);

    void SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void SetRho(const std::string& task_name, const double& rho);
    Eigen::VectorXd GetGoal(const std::string& task_name);
    double GetRho(const std::string& task_name);
    void PreUpdate() override;
    Eigen::MatrixXd GetBounds() const;

    bool IsValid() override;

    double GetScalarCost() const;
    Eigen::RowVectorXd GetScalarJacobian() const;
    double GetScalarTaskCost(const std::string& task_name) const;

    EndPoseTask cost;

    Eigen::MatrixXd W;
    TaskSpaceVector Phi;
    Hessian hessian;
    Eigen::MatrixXd jacobian;

    int length_Phi;
    int length_jacobian;
    int num_tasks;
};
typedef std::shared_ptr<exotica::BoundedEndPoseProblem> BoundedEndPoseProblemPtr;
}

#endif  // EXOTICA_CORE_BOUNDED_END_POSE_PROBLEM_H_
