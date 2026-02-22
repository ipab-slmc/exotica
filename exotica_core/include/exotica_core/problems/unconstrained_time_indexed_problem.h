//
// Copyright (c) 2019, University of Edinburgh
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

#ifndef EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_
#define EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_

#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/problems/abstract_time_indexed_problem.h>
#include <exotica_core/unconstrained_time_indexed_problem_initializer.h>

namespace exotica
{
/// \brief Unconstrained time-indexed problem.
class UnconstrainedTimeIndexedProblem : public AbstractTimeIndexedProblem, public Instantiable<UnconstrainedTimeIndexedProblemInitializer>
{
public:
    using AbstractTimeIndexedProblem::Update;

    UnconstrainedTimeIndexedProblem() = default;
    virtual ~UnconstrainedTimeIndexedProblem() = default;

    /// \brief Instantiates the problem from an Initializer
    void Instantiate(const UnconstrainedTimeIndexedProblemInitializer& init) override;

    /// \brief Updates internal variables before solving, e.g., after setting new values for Rho.
    void PreUpdate() override;

    /// \brief Updates an individual timestep from a given state vector
    /// \param x_in     State
    /// \param t        Timestep to update
    void Update(Eigen::VectorXdRefConst x_in, int t) override;

    // As this is an unconstrained problem, it is always valid.
    bool IsValid() override;

    // Delete bound constraints
    Eigen::MatrixXd GetBounds() const = delete;

    // Delete general constraints
    void SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0) = delete;
    Eigen::VectorXd GetGoalEQ(const std::string& task_name, int t = 0) = delete;
    void SetRhoEQ(const std::string& task_name, const double rho, int t = 0) = delete;
    double GetRhoEQ(const std::string& task_name, int t = 0) = delete;
    void SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0) = delete;
    Eigen::VectorXd GetGoalNEQ(const std::string& task_name, int t = 0) = delete;
    void SetRhoNEQ(const std::string& task_name, const double rho, int t = 0) = delete;
    double GetRhoNEQ(const std::string& task_name, int t = 0) = delete;
    Eigen::VectorXd GetEquality() const = delete;
    Eigen::VectorXd GetInequality() const = delete;
    Eigen::SparseMatrix<double> GetEqualityJacobian() const = delete;
    Eigen::SparseMatrix<double> GetInequalityJacobian() const = delete;
    std::vector<Eigen::Triplet<double>> GetEqualityJacobianTriplets() const = delete;
    int get_active_nonlinear_equality_constraints_dimension() const = delete;
    Eigen::VectorXd GetEquality(int t) const = delete;
    Eigen::MatrixXd GetEqualityJacobian(int t) const = delete;
    Eigen::VectorXd GetInequality(int t) const = delete;
    Eigen::MatrixXd GetInequalityJacobian(int t) const = delete;
    std::vector<Eigen::Triplet<double>> GetInequalityJacobianTriplets() const = delete;
    int get_active_nonlinear_inequality_constraints_dimension() const = delete;
    int get_joint_velocity_constraint_dimension() const = delete;
    Eigen::VectorXd GetJointVelocityConstraint() const = delete;
    Eigen::MatrixXd GetJointVelocityConstraintBounds() const = delete;
    std::vector<Eigen::Triplet<double>> GetJointVelocityConstraintJacobianTriplets() const = delete;
    Eigen::VectorXd GetJointVelocityLimits() const = delete;
    void SetJointVelocityLimits(const Eigen::VectorXd& qdot_max_in) = delete;

private:
    void ReinitializeVariables() override;
};
typedef std::shared_ptr<exotica::UnconstrainedTimeIndexedProblem> UnconstrainedTimeIndexedProblemPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_
