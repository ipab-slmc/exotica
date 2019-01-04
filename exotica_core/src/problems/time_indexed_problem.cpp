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

#include <exotica_core/problems/time_indexed_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedProblem", exotica::TimeIndexedProblem)

namespace exotica
{
TimeIndexedProblem::TimeIndexedProblem()
    : T_(0), tau_(0), w_scale_(0)
{
    flags_ = KIN_FK | KIN_J;
}

TimeIndexedProblem::~TimeIndexedProblem() = default;

Eigen::MatrixXd TimeIndexedProblem::GetBounds() const
{
    return scene_->GetKinematicTree().GetJointLimits();
}

void TimeIndexedProblem::Instantiate(TimeIndexedProblemInitializer& init)
{
    init_ = init;

    N = scene_->GetKinematicTree().GetNumControlledJoints();

    w_scale_ = init_.Wrate;
    W = Eigen::MatrixXd::Identity(N, N) * w_scale_;
    if (init_.W.rows() > 0)
    {
        if (init_.W.rows() == N)
        {
            W.diagonal() = init_.W * w_scale_;
        }
        else
        {
            ThrowNamed("W dimension mismatch! Expected " << N << ", got " << init_.W.rows());
        }
    }

    if (init.LowerBound.rows() == N)
    {
        scene_->GetKinematicTree().SetJointLimitsLower(init.LowerBound);
    }
    else if (init.LowerBound.rows() != 0)
    {
        ThrowNamed("Lower bound size incorrect! Expected " << N << " got " << init.LowerBound.rows());
    }
    if (init.UpperBound.rows() == N)
    {
        scene_->GetKinematicTree().SetJointLimitsUpper(init.UpperBound);
    }
    else if (init.UpperBound.rows() != 0)
    {
        ThrowNamed("Lower bound size incorrect! Expected " << N << " got " << init.UpperBound.rows());
    }

    use_bounds = init_.UseBounds;

    cost.Initialize(init_.Cost, shared_from_this(), cost_Phi);
    inequality.Initialize(init_.Inequality, shared_from_this(), inequality_Phi);
    equality.Initialize(init_.Equality, shared_from_this(), equality_Phi);

    T_ = init_.T;
    q_dot_max_ = init_.JointVelocityLimit;
    ApplyStartState(false);
    ReinitializeVariables();
}

void TimeIndexedProblem::ReinitializeVariables()
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

    if (flags_ & KIN_J) jacobian.assign(T_, Eigen::MatrixXd(length_jacobian, N));
    x.assign(T_, Eigen::VectorXd::Zero(N));
    xdiff.assign(T_, Eigen::VectorXd::Zero(N));
    if (flags_ & KIN_J_DOT)
    {
        Hessian Htmp;
        Htmp.setConstant(length_jacobian, Eigen::MatrixXd::Zero(N, N));
        hessian.assign(T_, Htmp);
    }

    // Set initial trajectory
    initial_trajectory_.resize(T_, scene_->GetControlledState());

    cost.ReinitializeVariables(T_, shared_from_this(), cost_Phi);
    inequality.ReinitializeVariables(T_, shared_from_this(), inequality_Phi);
    equality.ReinitializeVariables(T_, shared_from_this(), equality_Phi);
    PreUpdate();
}

void TimeIndexedProblem::SetT(const int& T_in)
{
    if (T_in <= 2)
    {
        ThrowNamed("Invalid number of timesteps: " << T_in);
    }
    T_ = T_in;
    ReinitializeVariables();
}

void TimeIndexedProblem::SetTau(const double& tau_in)
{
    if (tau_in <= 0.) ThrowPretty("tau_ is expected to be greater than 0. (tau_=" << tau_in << ")");
    tau_ = tau_in;
    ct = 1.0 / tau_ / T_;
    xdiff_max_ = q_dot_max_ * tau_;
}

void TimeIndexedProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    cost.UpdateS();
    inequality.UpdateS();
    equality.UpdateS();

    // Create a new set of kinematic solutions with the size of the trajectory
    // based on the lastest KinematicResponse in order to reflect model state
    // updates etc.
    kinematic_solutions_.clear();
    kinematic_solutions_.resize(T_);
    for (int i = 0; i < T_; ++i) kinematic_solutions_[i] = std::make_shared<KinematicResponse>(*scene_->GetKinematicTree().GetKinematicResponse());
}

void TimeIndexedProblem::SetInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in)
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

std::vector<Eigen::VectorXd> TimeIndexedProblem::GetInitialTrajectory()
{
    return initial_trajectory_;
}

double TimeIndexedProblem::GetDuration()
{
    return tau_ * static_cast<double>(T_);
}

void TimeIndexedProblem::Update(Eigen::VectorXdRefConst x_in, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }

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

double TimeIndexedProblem::GetScalarTaskCost(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return ct * cost.ydiff[t].transpose() * cost.S[t] * cost.ydiff[t];
}

Eigen::VectorXd TimeIndexedProblem::GetScalarTaskJacobian(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return cost.jacobian[t].transpose() * cost.S[t] * cost.ydiff[t] * 2.0 * ct;
}

double TimeIndexedProblem::GetScalarTransitionCost(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return ct * xdiff[t].transpose() * W * xdiff[t];
}

Eigen::VectorXd TimeIndexedProblem::GetScalarTransitionJacobian(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return 2.0 * ct * W * xdiff[t];
}

Eigen::VectorXd TimeIndexedProblem::GetEquality(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return equality.S[t] * equality.ydiff[t];
}

Eigen::MatrixXd TimeIndexedProblem::GetEqualityJacobian(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return equality.S[t] * equality.jacobian[t];
}

Eigen::VectorXd TimeIndexedProblem::GetInequality(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return inequality.S[t] * inequality.ydiff[t];
}

Eigen::MatrixXd TimeIndexedProblem::GetInequalityJacobian(int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return inequality.S[t] * inequality.jacobian[t];
}

void TimeIndexedProblem::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != cost.indexing[i].length) ThrowPretty("Expected length of " << cost.indexing[i].length << " and got " << goal.rows());
            cost.y[t].data.segment(cost.indexing[i].start, cost.indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::SetRho(const std::string& task_name, const double rho, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            cost.rho[t](cost.indexing[i].id) = rho;
            PreUpdate();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::GetGoal(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            return cost.y[t].data.segment(cost.indexing[i].start, cost.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::GetRho(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            return cost.rho[t](cost.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != equality.indexing[i].length) ThrowPretty("Expected length of " << equality.indexing[i].length << " and got " << goal.rows());
            equality.y[t].data.segment(equality.indexing[i].start, equality.indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::SetRhoEQ(const std::string& task_name, const double rho, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            equality.rho[t](equality.indexing[i].id) = rho;
            PreUpdate();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::GetGoalEQ(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            return equality.y[t].data.segment(equality.indexing[i].start, equality.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::GetRhoEQ(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            return equality.rho[t](equality.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != inequality.indexing[i].length) ThrowPretty("Expected length of " << inequality.indexing[i].length << " and got " << goal.rows());
            inequality.y[t].data.segment(inequality.indexing[i].start, inequality.indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::SetRhoNEQ(const std::string& task_name, const double rho, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            inequality.rho[t](inequality.indexing[i].id) = rho;
            PreUpdate();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::GetGoalNEQ(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            return inequality.y[t].data.segment(inequality.indexing[i].start, inequality.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::GetRhoNEQ(const std::string& task_name, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            return inequality.rho[t](inequality.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

bool TimeIndexedProblem::IsValid()
{
    bool succeeded = true;
    auto bounds = scene_->GetKinematicTree().GetJointLimits();

    // Check for every state
    for (unsigned int t = 0; t < T_; ++t)
    {
        // Check joint limits
        for (unsigned int i = 0; i < N; ++i)
        {
            if (x[t](i) < bounds(i, 0) || x[t](i) > bounds(i, 1))
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "State at timestep " << t << " is out of bounds");
                succeeded = false;
            }
        }

        // Check inequality constraints
        if (GetInequality(t).rows() > 0)
        {
            if (GetInequality(t).maxCoeff() > init_.InequalityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "Violated inequality constraints at timestep " << t << ": " << GetInequality(t).transpose());
                succeeded = false;
            }
        }

        // Check equality constraints
        if (GetEquality(t).rows() > 0)
        {
            if (GetEquality(t).norm() > init_.EqualityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "Violated equality constraints at timestep " << t << ": " << GetEquality(t).norm());
                succeeded = false;
            }
        }
    }

    return succeeded;
}
}
