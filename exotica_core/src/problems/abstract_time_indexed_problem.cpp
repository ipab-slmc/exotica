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

#include <exotica_core/problems/abstract_time_indexed_problem.h>
#include <exotica_core/setup.h>

#include <exotica_core/bounded_time_indexed_problem_initializer.h>
#include <exotica_core/time_indexed_problem_initializer.h>
#include <exotica_core/unconstrained_time_indexed_problem_initializer.h>

namespace exotica
{
template <typename InitializerType>
AbstractTimeIndexedProblem<InitializerType>::AbstractTimeIndexedProblem()
{
    flags_ = KIN_FK | KIN_J;
}

template <typename InitializerType>
AbstractTimeIndexedProblem<InitializerType>::~AbstractTimeIndexedProblem() = default;

template <typename InitializerType>
Eigen::MatrixXd AbstractTimeIndexedProblem<InitializerType>::GetBounds() const
{
    return scene_->GetKinematicTree().GetJointLimits();
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::ReinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem", "Initialize problem with T=" << T_);

    SetTau(init_.tau);

    num_tasks = tasks_.size();
    length_Phi = 0;
    length_jacobian = 0;
    TaskSpaceVector y_ref_;
    for (int i = 0; i < num_tasks; ++i)
    {
        AppendVector(y_ref_.map, tasks_[i]->GetLieGroupIndices());
        length_Phi += tasks_[i]->length;
        length_jacobian += tasks_[i]->length_jacobian;
    }

    y_ref_.SetZero(length_Phi);
    Phi.assign(T_, y_ref_);

    x.assign(T_, Eigen::VectorXd::Zero(N));
    xdiff.assign(T_, Eigen::VectorXd::Zero(N));
    if (flags_ & KIN_J) jacobian.assign(T_, Eigen::MatrixXd(length_jacobian, N));
    if (flags_ & KIN_J_DOT)
    {
        Hessian Htmp;
        Htmp.setConstant(length_jacobian, Eigen::MatrixXd::Zero(N, N));
        hessian.assign(T_, Htmp);
    }

    // Set initial trajectory
    initial_trajectory_.resize(T_, scene_->GetControlledState());

    // Initialize cost and constraints
    cost.ReinitializeVariables(T_, shared_from_this(), cost_Phi);
    inequality.ReinitializeVariables(T_, shared_from_this(), inequality_Phi);
    equality.ReinitializeVariables(T_, shared_from_this(), equality_Phi);

    // Initialize joint velocity constraint
    joint_velocity_constraint_dimension_ = N * (T_ - 1);
    joint_velocity_constraint_jacobian_triplets_.clear();
    // The Jacobian is constant so we can allocate the triplets here for the problem:
    typedef Eigen::Triplet<double> T;
    joint_velocity_constraint_jacobian_triplets_.reserve(joint_velocity_constraint_dimension_);
    for (int t = 1; t < T_; ++t)
    {
        for (int n = 0; n < N; ++n)
        {
            // x_t+1 ==> 1
            joint_velocity_constraint_jacobian_triplets_.emplace_back(T((t - 1) * N + n, (t - 1) * N + n, 1.0));

            if (t > 1)
            {
                // x_t => -1
                joint_velocity_constraint_jacobian_triplets_.emplace_back(T((t - 1) * N + n, (t - 2) * N + n, -1.0));
            }
        }
    }

    // Pre-update
    PreUpdate();
}

template <typename InitializerType>
inline void AbstractTimeIndexedProblem<InitializerType>::ValidateTimeIndex(int& t_in) const
{
    if (t_in >= T_ || t_in < -1)
    {
        ThrowPretty("Requested t=" << t_in << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t_in == -1)
    {
        t_in = (T_ - 1);
    }
}

template <typename InitializerType>
int AbstractTimeIndexedProblem<InitializerType>::GetT() const
{
    return T_;
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetT(const int T_in)
{
    if (T_in <= 2)
    {
        ThrowNamed("Invalid number of timesteps: " << T_in);
    }
    T_ = T_in;
    ReinitializeVariables();
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetTau() const
{
    return tau_;
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetTau(const double tau_in)
{
    if (tau_in <= 0.) ThrowPretty("tau_ is expected to be greater than 0. (tau_=" << tau_in << ")");
    tau_ = tau_in;
    ct = 1.0 / tau_ / T_;
    xdiff_max_ = q_dot_max_ * tau_;
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    cost.UpdateS();
    inequality.UpdateS();
    equality.UpdateS();

    // Update list of active equality/inequality constraints:
    active_nonlinear_equality_constraints_dimension_ = 0;
    active_nonlinear_inequality_constraints_dimension_ = 0;
    active_nonlinear_equality_constraints_.clear();
    active_nonlinear_inequality_constraints_.clear();
    for (int t = 1; t < T_; ++t)
    {
        for (const TaskIndexing& task : equality.indexing)
        {
            if (equality.rho[t](task.id) != 0.0)
            {
                active_nonlinear_equality_constraints_.emplace_back(std::make_pair(t, task.id));
                active_nonlinear_equality_constraints_dimension_ += task.length_jacobian;
            }
        }

        for (const TaskIndexing& task : inequality.indexing)
        {
            if (inequality.rho[t](task.id) != 0.0)
            {
                active_nonlinear_inequality_constraints_.emplace_back(std::make_pair(t, task.id));
                active_nonlinear_inequality_constraints_dimension_ += task.length_jacobian;
            }
        }
    }

    // Create a new set of kinematic solutions with the size of the trajectory
    // based on the lastest KinematicResponse in order to reflect model state
    // updates etc.
    kinematic_solutions_.clear();
    kinematic_solutions_.resize(T_);
    for (int i = 0; i < T_; ++i) kinematic_solutions_[i] = std::make_shared<KinematicResponse>(*scene_->GetKinematicTree().GetKinematicResponse());
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in)
{
    if (q_init_in.size() != T_)
        ThrowPretty("Expected initial trajectory of length "
                    << T_ << " but got " << q_init_in.size());
    if (q_init_in[0].rows() != N)
        ThrowPretty("Expected states to have " << N << " rows but got "
                                               << q_init_in[0].rows());

    initial_trajectory_ = q_init_in;
    SetStartState(q_init_in[0]);
}

template <typename InitializerType>
std::vector<Eigen::VectorXd> AbstractTimeIndexedProblem<InitializerType>::GetInitialTrajectory() const
{
    return initial_trajectory_;
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetDuration() const
{
    return tau_ * static_cast<double>(T_);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::Update(Eigen::VectorXdRefConst x_trajectory_in)
{
    if (x_trajectory_in.size() != (T_ - 1) * N)
        ThrowPretty("To update using the trajectory Update method, please use a trajectory of size N x (T-1) (" << N * (T_ - 1) << "), given: " << x_trajectory_in.size());

    for (int t = 1; t < T_; ++t)
    {
        Update(x_trajectory_in.segment((t - 1) * N, N), t);
    }
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::Update(Eigen::VectorXdRefConst x_in, int t)
{
    ValidateTimeIndex(t);

    x[t] = x_in;

    // Set the corresponding KinematicResponse for KinematicTree in order to
    // have Kinematics elements updated based in x_in.
    scene_->GetKinematicTree().SetKinematicResponse(kinematic_solutions_[t]);

    // Pass the corresponding number of relevant task kinematics to the TaskMaps
    // via the PlanningProblem::updateMultipleTaskKinematics method. For now we
    // support passing _two_ timesteps - this can be easily changed later on.
    std::vector<std::shared_ptr<KinematicResponse>> kinematics_solutions{kinematic_solutions_[t]};

    // If the current timestep is 0, pass the 0th timestep's response twice.
    // Otherwise pass the (t-1)th response.
    kinematics_solutions.emplace_back((t == 0) ? kinematic_solutions_[t] : kinematic_solutions_[t - 1]);

    // Actually update the tasks' kinematics mappings.
    PlanningProblem::updateMultipleTaskKinematics(kinematics_solutions);

    scene_->Update(x_in, static_cast<double>(t) * tau_);
    Phi[t].SetZero(length_Phi);
    if (flags_ & KIN_J) jacobian[t].setZero();
    if (flags_ & KIN_J_DOT)
        for (int i = 0; i < length_jacobian; ++i) hessian[t](i).setZero();
    for (int i = 0; i < num_tasks; ++i)
    {
        // Only update TaskMap if rho is not 0
        if (tasks_[i]->is_used)
        {
            if (flags_ & KIN_J_DOT)
            {
                tasks_[i]->Update(x[t], Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length), jacobian[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian), hessian[t].segment(tasks_[i]->start, tasks_[i]->length));
            }
            else if (flags_ & KIN_J)
            {
                tasks_[i]->Update(x[t], Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length), jacobian[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else
            {
                tasks_[i]->Update(x[t], Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length));
            }
        }
    }
    if (flags_ & KIN_J_DOT)
    {
        cost.Update(Phi[t], jacobian[t], hessian[t], t);
        inequality.Update(Phi[t], jacobian[t], hessian[t], t);
        equality.Update(Phi[t], jacobian[t], hessian[t], t);
    }
    else if (flags_ & KIN_J)
    {
        cost.Update(Phi[t], jacobian[t], t);
        inequality.Update(Phi[t], jacobian[t], t);
        equality.Update(Phi[t], jacobian[t], t);
    }
    else
    {
        cost.Update(Phi[t], t);
        inequality.Update(Phi[t], t);
        equality.Update(Phi[t], t);
    }
    if (t > 0) xdiff[t] = x[t] - x[t - 1];
    ++number_of_problem_updates_;
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::get_ct() const
{
    return ct;
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetCost() const
{
    double cost = 0.0;
    for (int t = 1; t < T_; ++t)
    {
        cost += GetScalarTaskCost(t) + GetScalarTransitionCost(t);
    }
    return cost;
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetCostJacobian() const
{
    Eigen::VectorXd jac = Eigen::VectorXd::Zero(N * (T_ - 1));
    for (int t = 1; t < T_; ++t)
    {
        jac.segment((t - 1) * N, N) += GetScalarTaskJacobian(t) + GetScalarTransitionJacobian(t);
        if (t > 1) jac.segment((t - 2) * N, N) -= GetScalarTransitionJacobian(t);
    }
    return jac;
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetScalarTaskCost(int t) const
{
    ValidateTimeIndex(t);
    return ct * cost.ydiff[t].transpose() * cost.S[t] * cost.ydiff[t];
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetScalarTaskJacobian(int t) const
{
    ValidateTimeIndex(t);
    return cost.jacobian[t].transpose() * cost.S[t] * cost.ydiff[t] * 2.0 * ct;
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetScalarTransitionCost(int t) const
{
    ValidateTimeIndex(t);
    return ct * xdiff[t].transpose() * W * xdiff[t];
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetScalarTransitionJacobian(int t) const
{
    ValidateTimeIndex(t);
    return 2.0 * ct * W * xdiff[t];
}

template <typename InitializerType>
int AbstractTimeIndexedProblem<InitializerType>::get_active_nonlinear_equality_constraints_dimension() const
{
    return active_nonlinear_equality_constraints_dimension_;
}

template <typename InitializerType>
int AbstractTimeIndexedProblem<InitializerType>::get_active_nonlinear_inequality_constraints_dimension() const
{
    return active_nonlinear_inequality_constraints_dimension_;
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetEquality() const
{
    Eigen::VectorXd eq = Eigen::VectorXd::Zero(active_nonlinear_equality_constraints_dimension_);
    int start = 0;
    for (const auto& constraint : active_nonlinear_equality_constraints_)
    {
        // First is timestep, second is task id
        const TaskIndexing task = equality.indexing[constraint.second];
        eq.segment(start, task.length_jacobian) = equality.rho[constraint.first](task.id) * equality.ydiff[constraint.first].segment(task.start_jacobian, task.length_jacobian);
        start += task.length_jacobian;
    }

    return eq;
}

template <typename InitializerType>
Eigen::SparseMatrix<double> AbstractTimeIndexedProblem<InitializerType>::GetEqualityJacobian() const
{
    Eigen::SparseMatrix<double> jac(active_nonlinear_equality_constraints_dimension_, N * (T_ - 1));
    std::vector<Eigen::Triplet<double>> triplet_list = GetEqualityJacobianTriplets();
    jac.setFromTriplets(triplet_list.begin(), triplet_list.end());
    return jac;
}

template <typename InitializerType>
std::vector<Eigen::Triplet<double>> AbstractTimeIndexedProblem<InitializerType>::GetEqualityJacobianTriplets() const
{
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplet_list;
    triplet_list.reserve(active_nonlinear_equality_constraints_dimension_ * N);
    int start = 0;
    for (const auto& constraint : active_nonlinear_equality_constraints_)
    {
        // First is timestep, second is task id
        const TaskIndexing task = equality.indexing[constraint.second];

        const int row_start = start;
        const int row_length = task.length_jacobian;
        const int column_start = (constraint.first - 1) * N;  // (t - 1) * N
        const int column_length = N;

        const Eigen::MatrixXd jacobian = equality.rho[constraint.first](task.id) * equality.jacobian[constraint.first].middleRows(task.start_jacobian, task.length_jacobian);
        int row_idx = 0;
        for (int row = row_start; row < row_start + row_length; ++row)
        {
            int column_idx = 0;
            for (int column = column_start; column < column_start + column_length; ++column)
            {
                triplet_list.emplace_back(Eigen::Triplet<double>(row, column, jacobian(row_idx, column_idx)));
                ++column_idx;
            }
            ++row_idx;
        }

        start += task.length_jacobian;
    }
    return triplet_list;
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetEquality(int t) const
{
    ValidateTimeIndex(t);
    return equality.S[t] * equality.ydiff[t];
}

template <typename InitializerType>
Eigen::MatrixXd AbstractTimeIndexedProblem<InitializerType>::GetEqualityJacobian(int t) const
{
    ValidateTimeIndex(t);
    return equality.S[t] * equality.jacobian[t];
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetInequality() const
{
    Eigen::VectorXd neq = Eigen::VectorXd::Zero(active_nonlinear_inequality_constraints_dimension_);
    int start = 0;
    for (const auto& constraint : active_nonlinear_inequality_constraints_)
    {
        // First is timestep, second is task id
        const TaskIndexing task = inequality.indexing[constraint.second];
        neq.segment(start, task.length_jacobian) = inequality.rho[constraint.first](task.id) * inequality.ydiff[constraint.first].segment(task.start_jacobian, task.length_jacobian);
        start += task.length_jacobian;
    }

    return neq;
}

template <typename InitializerType>
Eigen::SparseMatrix<double> AbstractTimeIndexedProblem<InitializerType>::GetInequalityJacobian() const
{
    Eigen::SparseMatrix<double> jac(active_nonlinear_inequality_constraints_dimension_, N * (T_ - 1));
    std::vector<Eigen::Triplet<double>> triplet_list = GetInequalityJacobianTriplets();
    jac.setFromTriplets(triplet_list.begin(), triplet_list.end());
    return jac;
}

template <typename InitializerType>
std::vector<Eigen::Triplet<double>> AbstractTimeIndexedProblem<InitializerType>::GetInequalityJacobianTriplets() const
{
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplet_list;
    triplet_list.reserve(active_nonlinear_inequality_constraints_dimension_ * N);
    int start = 0;
    for (const auto& constraint : active_nonlinear_inequality_constraints_)
    {
        // First is timestep, second is task id
        const TaskIndexing task = inequality.indexing[constraint.second];

        const int row_start = start;
        const int row_length = task.length_jacobian;
        const int column_start = (constraint.first - 1) * N;  // (t - 1) * N
        const int column_length = N;

        const Eigen::MatrixXd jacobian = inequality.rho[constraint.first](task.id) * inequality.jacobian[constraint.first].middleRows(task.start_jacobian, task.length_jacobian);
        int row_idx = 0;
        for (int row = row_start; row < row_start + row_length; ++row)
        {
            int column_idx = 0;
            for (int column = column_start; column < column_start + column_length; ++column)
            {
                triplet_list.emplace_back(Eigen::Triplet<double>(row, column, jacobian(row_idx, column_idx)));
                ++column_idx;
            }
            ++row_idx;
        }

        start += task.length_jacobian;
    }
    return triplet_list;
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetInequality(int t) const
{
    ValidateTimeIndex(t);
    return inequality.S[t] * inequality.ydiff[t];
}

template <typename InitializerType>
Eigen::MatrixXd AbstractTimeIndexedProblem<InitializerType>::GetInequalityJacobian(int t) const
{
    ValidateTimeIndex(t);
    return inequality.S[t] * inequality.jacobian[t];
}

template <typename InitializerType>
int AbstractTimeIndexedProblem<InitializerType>::get_joint_velocity_constraint_dimension() const
{
    return joint_velocity_constraint_dimension_;
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetJointVelocityLimits() const
{
    return q_dot_max_;
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetJointVelocityLimits(const Eigen::VectorXd& qdot_max_in)
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

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetJointVelocityConstraint() const
{
    Eigen::VectorXd g(joint_velocity_constraint_dimension_);
    for (int t = 1; t < T_; ++t)
    {
        g.segment((t - 1) * N, N) = xdiff[t];
    }
    return g;
}

template <typename InitializerType>
Eigen::MatrixXd AbstractTimeIndexedProblem<InitializerType>::GetJointVelocityConstraintBounds() const
{
    Eigen::MatrixXd b(joint_velocity_constraint_dimension_, 2);
    for (int t = 1; t < T_; ++t)
    {
        // As we are not including the T=0 in the optimization problem, we cannot
        // define a transition (xdiff) constraint for the 0th-to-1st timestep
        // directly - we need to include the constant x_{t=0} values in the ``b``
        //  element of the linear constraint, i.e., as an additional offset in bounds.
        if (t == 1)
        {
            b.block((t - 1) * N, 0, N, 1) = -xdiff_max_ + initial_trajectory_[0];
            b.block((t - 1) * N, 1, N, 1) = xdiff_max_ + initial_trajectory_[0];
        }
        else
        {
            b.block((t - 1) * N, 0, N, 1) = -xdiff_max_;
            b.block((t - 1) * N, 1, N, 1) = xdiff_max_;
        }
    }
    return b;
}

template <typename InitializerType>
std::vector<Eigen::Triplet<double>> AbstractTimeIndexedProblem<InitializerType>::GetJointVelocityConstraintJacobianTriplets() const
{
    // The Jacobian is constant - and thus cached (in ReinitializeVariables)
    return joint_velocity_constraint_jacobian_triplets_;
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    cost.SetGoal(task_name, goal, t);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetRho(const std::string& task_name, const double rho, int t)
{
    cost.SetRho(task_name, rho, t);
    PreUpdate();
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetGoal(const std::string& task_name, int t)
{
    return cost.GetGoal(task_name, t);
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetRho(const std::string& task_name, int t)
{
    return cost.GetRho(task_name, t);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    equality.SetGoal(task_name, goal, t);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetRhoEQ(const std::string& task_name, const double rho, int t)
{
    equality.SetRho(task_name, rho, t);
    PreUpdate();
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetGoalEQ(const std::string& task_name, int t)
{
    return equality.GetGoal(task_name, t);
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetRhoEQ(const std::string& task_name, int t)
{
    return equality.GetRho(task_name, t);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    inequality.SetGoal(task_name, goal, t);
}

template <typename InitializerType>
void AbstractTimeIndexedProblem<InitializerType>::SetRhoNEQ(const std::string& task_name, const double rho, int t)
{
    inequality.SetRho(task_name, rho, t);
    PreUpdate();
}

template <typename InitializerType>
Eigen::VectorXd AbstractTimeIndexedProblem<InitializerType>::GetGoalNEQ(const std::string& task_name, int t)
{
    return inequality.GetGoal(task_name, t);
}

template <typename InitializerType>
double AbstractTimeIndexedProblem<InitializerType>::GetRhoNEQ(const std::string& task_name, int t)
{
    return inequality.GetRho(task_name, t);
}

template class AbstractTimeIndexedProblem<TimeIndexedProblemInitializer>;
template class AbstractTimeIndexedProblem<BoundedTimeIndexedProblemInitializer>;
template class AbstractTimeIndexedProblem<UnconstrainedTimeIndexedProblemInitializer>;
}  // namespace exotica
