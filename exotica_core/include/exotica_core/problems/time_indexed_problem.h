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

#include <Eigen/Sparse>

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

    /// \brief Instantiates the problem from an Initializer
    void Instantiate(TimeIndexedProblemInitializer& init) override;

    /// \brief Returns the duration of the trajectory (T * tau).
    double GetDuration() const;

    /// \brief Updates the entire problem from a given trajectory (e.g., used in an optimization solver)
    /// \param x_trajectory_in      Trajectory flattened as a vector; expects dimension: (T - 1) * N
    void Update(Eigen::VectorXdRefConst x_trajectory_in);

    /// \brief Updates an individual timestep from a given state vector
    /// \param x_in     State
    /// \param t        Timestep to update
    void Update(Eigen::VectorXdRefConst x_in, int t);

    /// \brief Evaluates whether the problem is valid, i.e., all bound and general constraints are satisfied.
    bool IsValid() override;

    /// \brief Returns the initial trajectory/seed
    std::vector<Eigen::VectorXd> GetInitialTrajectory() const;

    /// \brief Sets the initial trajectory/seed
    /// \param q_init_in    Vector of states. Expects T states of dimension N.
    void SetInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in);

    /// \brief Updates internal variables before solving, e.g., after setting new values for Rho.
    void PreUpdate() override;

    /// \brief Sets goal for a given task at a given timestep (cost task).
    /// \param task_name    Name of task
    /// \param goal         Goal
    /// \param t            Timestep
    void SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);

    /// \brief Returns the goal for a given task at a given timestep (cost task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    Eigen::VectorXd GetGoal(const std::string& task_name, int t = 0);

    /// \brief Sets Rho for a given task at a given timestep (cost task).
    /// \param task_name    Name of task
    /// \param rho          Rho (scaling/precision)
    /// \param t            Timestep
    void SetRho(const std::string& task_name, const double rho, int t = 0);

    /// \brief Returns the precision (Rho) for a given task at a given timestep (cost task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    double GetRho(const std::string& task_name, int t = 0);

    /// \brief Sets goal for a given task at a given timestep (equality task).
    /// \param task_name    Name of task
    /// \param goal         Goal
    /// \param t            Timestep
    void SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);

    /// \brief Returns the goal for a given task at a given timestep (equality task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    Eigen::VectorXd GetGoalEQ(const std::string& task_name, int t = 0);

    /// \brief Sets Rho for a given task at a given timestep (equality task).
    /// \param task_name    Name of task
    /// \param rho          Rho (scaling/precision)
    /// \param t            Timestep
    void SetRhoEQ(const std::string& task_name, const double rho, int t = 0);

    /// \brief Returns the precision (Rho) for a given task at a given timestep (equality task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    double GetRhoEQ(const std::string& task_name, int t = 0);

    /// \brief Sets goal for a given task at a given timestep (inequality task).
    /// \param task_name    Name of task
    /// \param goal         Goal
    /// \param t            Timestep
    void SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);

    /// \brief Returns the goal for a given task at a given timestep (goal task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    Eigen::VectorXd GetGoalNEQ(const std::string& task_name, int t = 0);

    /// \brief Sets Rho for a given task at a given timestep (inequality task).
    /// \param task_name    Name of task
    /// \param rho          Rho (scaling/precision)
    /// \param t            Timestep
    void SetRhoNEQ(const std::string& task_name, const double rho, int t = 0);

    /// \brief Returns the precision (Rho) for a given task at a given timestep (equality task).
    /// \param task_name    Name of task
    /// \param t            Timestep
    double GetRhoNEQ(const std::string& task_name, int t = 0);

    /// \brief Returns the joint bounds (first column lower, second column upper).
    Eigen::MatrixXd GetBounds() const;

    /// \brief Returns the number of timesteps in the trajectory.
    int GetT() const { return T_; }
    /// \brief Sets the number of timesteps in the trajectory. Note: Rho/Goal need to be updated for every timestep after calling this method.
    void SetT(const int T_in);

    /// \brief Returns the time discretization tau for the trajectory.
    double GetTau() const { return tau_; }
    /// \brief Sets the time discretization tau for the trajectory.
    void SetTau(const double tau_in);

    /// \brief Returns the scalar task cost at timestep t
    double GetScalarTaskCost(int t) const;

    /// \brief Returns the Jacobian of the scalar task cost at timestep t
    Eigen::VectorXd GetScalarTaskJacobian(int t) const;

    /// \brief Returns the scalar transition cost (x^T*W*x) at timestep t
    double GetScalarTransitionCost(int t) const;

    /// \brief Returns the Jacobian of the transition cost at timestep t
    Eigen::VectorXd GetScalarTransitionJacobian(int t) const;

    /// \brief Returns the scalar cost for the entire trajectory (both task and transition cost).
    double GetCost() const;

    /// \brief Returns the Jacobian of the scalar cost over the entire trajectory (Jacobian of GetCost).
    Eigen::VectorXd GetCostJacobian() const;

    /// \brief Returns the equality constraint values for the entire trajectory.
    Eigen::VectorXd GetEquality() const;

    /// \brief Returns the inequality constraint values for the entire trajectory.
    Eigen::VectorXd GetInequality() const;

    /// \brief Returns the sparse Jacobian matrix of the equality constraints over the entire trajectory.
    Eigen::SparseMatrix<double> GetEqualityJacobian() const;

    /// \brief Returns the sparse Jacobian matrix of the inequality constraints over the entire trajectory.
    Eigen::SparseMatrix<double> GetInequalityJacobian() const;

    /// \brief Returns a vector of triplets to fill a sparse Jacobian for the equality constraints.
    std::vector<Eigen::Triplet<double>> GetEqualityJacobianTriplets() const;

    /// \brief Returns the dimension of the active equality constraints.
    int get_active_nonlinear_equality_constraints_dimension() const { return active_nonlinear_equality_constraints_dimension_; }
    /// \brief Returns the value of the equality constraints at timestep t.
    Eigen::VectorXd GetEquality(int t) const;

    /// \brief Returns the Jacobian of the equality constraints at timestep t.
    Eigen::MatrixXd GetEqualityJacobian(int t) const;

    /// \brief Returns the value of the inequality constraints at timestep t.
    Eigen::VectorXd GetInequality(int t) const;

    /// \brief Returns the Jacobian of the inequality constraints at timestep t.
    Eigen::MatrixXd GetInequalityJacobian(int t) const;

    /// \brief Returns a vector of triplets to fill a sparse Jacobian for the inequality constraints.
    std::vector<Eigen::Triplet<double>> GetInequalityJacobianTriplets() const;

    /// \brief Returns the dimension of the active inequality constraints.
    int get_active_nonlinear_inequality_constraints_dimension() const { return active_nonlinear_inequality_constraints_dimension_; }
    /// \brief Returns the dimension of the joint velocity constraint (linear inequality).
    int get_joint_velocity_constraint_dimension() const { return joint_velocity_constraint_dimension_; }
    /// \brief Returns the joint velocity constraint inequality terms (linear).
    Eigen::VectorXd GetJointVelocityConstraint() const;

    /// \brief Returns the joint velocity constraint bounds (constant terms).
    Eigen::MatrixXd GetJointVelocityConstraintBounds() const;

    /// \brief Returns the joint velocity constraint Jacobian as triplets.
    std::vector<Eigen::Triplet<double>> GetJointVelocityConstraintJacobianTriplets() const;

    /// \brief Returns the per-DoF joint velocity limit vector.
    Eigen::VectorXd GetJointVelocityLimits() const { return q_dot_max_; }
    /// \brief Sets the joint velocity limits. Supports N- and 1-dimensional vectors.
    void SetJointVelocityLimits(const Eigen::VectorXd& qdot_max_in)
    {
        if (qdot_max_in.size() == N)
        {
            q_dot_max_ = qdot_max_in;
        }
        else if (qdot_max_in.size() == 1)
        {
            q_dot_max_ = qdot_max_in(0) * Eigen::VectorXd::Ones(N);
        }
        else
        {
            ThrowPretty("Received size " << qdot_max_in.size() << " but expected 1 or " << N);
        }
        xdiff_max_ = q_dot_max_ * tau_;
    }

    /// \brief Returns the maximum diff between two timesteps for each dimension in the configuration vector x.
    Eigen::VectorXd GetXdiffMax() const { return xdiff_max_; }
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

    std::vector<Eigen::VectorXd> x;      ///< Current internal problem state
    std::vector<Eigen::VectorXd> xdiff;  // equivalent to dx = x(t)-x(t-1)

    int length_Phi;
    int length_jacobian;
    int num_tasks;
    bool use_bounds;

private:
    void ReinitializeVariables();

    /// \brief Checks the desired time index for bounds and supports -1 indexing.
    inline void ValidateTimeIndex(int& t_in) const;

    int T_ = 0;       //!< Number of time steps
    double tau_ = 0;  //!< Time step duration

    Eigen::VectorXd q_dot_max_;  //!< Joint velocity limit (rad/s)
    Eigen::VectorXd xdiff_max_;  //!< Maximum change in the variables in a single timestep tau_. Gets set/updated via SetTau().

    double w_scale_ = 1.0;  //!< Kinematic system transition error covariance multiplier (constant throughout the trajectory)

    std::vector<Eigen::VectorXd> initial_trajectory_;
    TimeIndexedProblemInitializer init_;

    std::vector<std::shared_ptr<KinematicResponse>> kinematic_solutions_;

    // The first element in the pair is the timestep (t) and the second element is the task.id (id).
    std::vector<std::pair<int, int>> active_nonlinear_equality_constraints_;
    std::vector<std::pair<int, int>> active_nonlinear_inequality_constraints_;
    int active_nonlinear_equality_constraints_dimension_ = 0;
    int active_nonlinear_inequality_constraints_dimension_ = 0;

    // Terms related with the joint velocity constraint - the Jacobian triplets are constant so can be cached.
    int joint_velocity_constraint_dimension_ = 0;
    std::vector<Eigen::Triplet<double>> joint_velocity_constraint_jacobian_triplets_;
};

typedef std::shared_ptr<exotica::TimeIndexedProblem> TimeIndexedProblemPtr;
}

#endif  // EXOTICA_CORE_TIME_INDEXED_PROBLEM_H_
