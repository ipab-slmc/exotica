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

#ifndef EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_
#define EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_

#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/unconstrained_time_indexed_problem_initializer.h>

namespace exotica
{
class UnconstrainedTimeIndexedProblem : public PlanningProblem, public Instantiable<UnconstrainedTimeIndexedProblemInitializer>
{
public:
    UnconstrainedTimeIndexedProblem();
    virtual ~UnconstrainedTimeIndexedProblem();
    virtual void Instantiate(UnconstrainedTimeIndexedProblemInitializer& init);
    double GetDuration();
    void Update(Eigen::VectorXdRefConst x_in, int t);
    void SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void SetRho(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd GetGoal(const std::string& task_name, int t = 0);
    double GetRho(const std::string& task_name, int t = 0);
    std::vector<Eigen::VectorXd> GetInitialTrajectory();
    void SetInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in);
    void PreUpdate() override;

    int GetT() const { return T_; }
    void SetT(const int& T_in);

    double GetTau() const { return tau_; }
    void SetTau(const double& tau_in);

    double GetScalarTaskCost(int t);
    Eigen::VectorXd GetScalarTaskJacobian(int t);
    double GetScalarTransitionCost(int t);
    Eigen::VectorXd GetScalarTransitionJacobian(int t);

    std::vector<std::shared_ptr<KinematicResponse>> GetKinematicSolutions() { return kinematic_solutions_; }
    double ct;  //!< Normalisation of scalar cost and Jacobian over trajectory length
    TimeIndexedTask cost;
    Eigen::MatrixXd W;

    std::vector<TaskSpaceVector> Phi;
    std::vector<Eigen::MatrixXd> jacobian;
    std::vector<Hessian> hessian;

    std::vector<Eigen::VectorXd> x;      // current internal problem state
    std::vector<Eigen::VectorXd> xdiff;  // equivalent to dx = x(t)-x(t-1)

    bool IsValid() override { return true; }
    int length_Phi;
    int length_jacobian;
    int num_tasks;

    TaskSpaceVector cost_Phi;  // passed to the TimeIndexedTask, needs to be kept for reinitialisation

private:
    void ReinitializeVariables();

    int T_;       //!< Number of time steps
    double tau_;  //!< Time step duration

    double w_scale_;  //!< Kinematic system transition error covariance multiplier (constant throughout the trajectory)

    std::vector<Eigen::VectorXd> initial_trajectory_;
    UnconstrainedTimeIndexedProblemInitializer init_;
    TaskSpaceVector y_ref_;  //!< Stores task Phi reference value, to be assigned to Phi

    std::vector<std::shared_ptr<KinematicResponse>> kinematic_solutions_;
};

typedef std::shared_ptr<exotica::UnconstrainedTimeIndexedProblem> UnconstrainedTimeIndexedProblemPtr;
}

#endif  // EXOTICA_CORE_UNCONSTRAINED_TIME_INDEXED_PROBLEM_H_
