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

#ifndef EXOTICA_CORE_END_POSE_PROBLEM_H_
#define EXOTICA_CORE_END_POSE_PROBLEM_H_

#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/end_pose_problem_initializer.h>

namespace exotica
{
/// \brief Arbitrarily constrained end-pose problem implementation
class EndPoseProblem : public PlanningProblem, public Instantiable<EndPoseProblemInitializer>
{
public:
    EndPoseProblem();
    virtual ~EndPoseProblem();

    virtual void Instantiate(const EndPoseProblemInitializer& init);
    void Update(Eigen::VectorXdRefConst x);
    bool IsValid() override;

    void SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void SetRho(const std::string& task_name, const double& rho);
    Eigen::VectorXd GetGoal(const std::string& task_name);
    double GetRho(const std::string& task_name);
    void SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void SetRhoEQ(const std::string& task_name, const double& rho);
    Eigen::VectorXd GetGoalEQ(const std::string& task_name);
    double GetRhoEQ(const std::string& task_name);
    void SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void SetRhoNEQ(const std::string& task_name, const double& rho);
    Eigen::VectorXd GetGoalNEQ(const std::string& task_name);
    double GetRhoNEQ(const std::string& task_name);
    void PreUpdate() override;
    Eigen::MatrixXd GetBounds() const;

    double GetScalarCost();
    Eigen::RowVectorXd GetScalarJacobian();
    double GetScalarTaskCost(const std::string& task_name) const;
    Eigen::VectorXd GetEquality();
    Eigen::MatrixXd GetEqualityJacobian();
    Eigen::VectorXd GetInequality();
    Eigen::MatrixXd GetInequalityJacobian();

    EndPoseTask cost;
    EndPoseTask inequality;
    EndPoseTask equality;

    Eigen::MatrixXd W;
    TaskSpaceVector Phi;
    Eigen::MatrixXd jacobian;
    Hessian hessian;

    int length_Phi;
    int length_jacobian;
    int num_tasks;
    bool use_bounds;
};
typedef std::shared_ptr<exotica::EndPoseProblem> EndPoseProblemPtr;
}

#endif  // EXOTICA_CORE_END_POSE_PROBLEM_H_
