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

#ifndef EXOTICA_CORE_TIME_INDEXED_PROBLEM_H_
#define EXOTICA_CORE_TIME_INDEXED_PROBLEM_H_

#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/time_indexed_problem_initializer.h>

namespace exotica
{
class TimeIndexedProblem : public PlanningProblem, public Instantiable<TimeIndexedProblemInitializer>
{
public:
    TimeIndexedProblem();
    virtual ~TimeIndexedProblem();
    void Instantiate(TimeIndexedProblemInitializer& init) override;
    double GetDuration();
    void Update(Eigen::VectorXdRefConst x, int t);
    bool IsValid() override;
    std::vector<Eigen::VectorXd> GetInitialTrajectory();
    void SetInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in);
    void PreUpdate() override;
    void SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void SetRho(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd GetGoal(const std::string& task_name, int t = 0);
    double GetRho(const std::string& task_name, int t = 0);
    void SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void SetRhoEQ(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd GetGoalEQ(const std::string& task_name, int t = 0);
    double GetRhoEQ(const std::string& task_name, int t = 0);
    void SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void SetRhoNEQ(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd GetGoalNEQ(const std::string& task_name, int t = 0);
    double GetRhoNEQ(const std::string& task_name, int t = 0);
    Eigen::MatrixXd GetBounds() const;

    int GetT() const { return T_; }
    void SetT(const int& T_in);

    double GetTau() const { return tau_; }
    void SetTau(const double& tau_in);

    double GetScalarTaskCost(int t);
    Eigen::VectorXd GetScalarTaskJacobian(int t);
    double GetScalarTransitionCost(int t);
    Eigen::VectorXd GetScalarTransitionJacobian(int t);

    Eigen::VectorXd GetEquality(int t);
    Eigen::MatrixXd GetEqualityJacobian(int t);
    Eigen::VectorXd GetInequality(int t);
    Eigen::MatrixXd GetInequalityJacobian(int t);

    double GetJointVelocityLimit() { return q_dot_max_; }
    void SetJointVelocityLimit(const double& qdot_max_in)
    {
        q_dot_max_ = qdot_max_in;
        xdiff_max_ = q_dot_max_ * tau_;
    }
    double GetXdiffMax() { return xdiff_max_; }
    double ct;  //!< Normalisation of scalar cost and Jacobian over trajectory length
    TimeIndexedTask cost;
    TimeIndexedTask inequality;
    TimeIndexedTask equality;

    TaskSpaceVector cost_Phi;
    TaskSpaceVector inequality_Phi;
    TaskSpaceVector equality_Phi;

    Eigen::MatrixXd W;  // TODO(wxm): Make private and add getter and setter (#209)

    std::vector<TaskSpaceVector> Phi;
    std::vector<Eigen::MatrixXd> jacobian;
    std::vector<Hessian> hessian;

    std::vector<Eigen::VectorXd> x;      // current internal problem state
    std::vector<Eigen::VectorXd> xdiff;  // equivalent to dx = x(t)-x(t-1)

    int length_Phi;
    int length_jacobian;
    int num_tasks;
    bool use_bounds;

private:
    void ReinitializeVariables();

    int T_;       //!< Number of time steps
    double tau_;  //!< Time step duration

    double q_dot_max_;  //!< Joint velocity limit (rad/s)
    double xdiff_max_;  //!< Maximum change in the variables in a single timestep tau_. Gets set/updated via SetTau().

    double w_scale_;  //!< Kinematic system transition error covariance multiplier (constant throughout the trajectory)

    std::vector<Eigen::VectorXd> initial_trajectory_;
    TimeIndexedProblemInitializer init_;

    std::vector<std::shared_ptr<KinematicResponse>> kinematic_solutions_;
};

typedef std::shared_ptr<exotica::TimeIndexedProblem> TimeIndexedProblemPtr;
}

#endif  // EXOTICA_CORE_TIME_INDEXED_PROBLEM_H_
