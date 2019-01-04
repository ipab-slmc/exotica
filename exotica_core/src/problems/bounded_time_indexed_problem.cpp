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

#include <exotica_core/problems/bounded_time_indexed_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("BoundedTimeIndexedProblem", exotica::BoundedTimeIndexedProblem)

namespace exotica
{
BoundedTimeIndexedProblem::BoundedTimeIndexedProblem()
    : T_(0), tau_(0), w_scale_(0)
{
    flags_ = KIN_FK | KIN_J;
}

BoundedTimeIndexedProblem::~BoundedTimeIndexedProblem() = default;

Eigen::MatrixXd BoundedTimeIndexedProblem::GetBounds() const
{
    return scene_->GetKinematicTree().GetJointLimits();
}

void BoundedTimeIndexedProblem::Instantiate(BoundedTimeIndexedProblemInitializer& init)
{
    parameters = init;

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

    cost.Initialize(parameters.Cost, shared_from_this(), cost_Phi);

    T_ = parameters.T;
    ApplyStartState(false);
    ReinitializeVariables();
}

void BoundedTimeIndexedProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    cost.UpdateS();
}

void BoundedTimeIndexedProblem::SetInitialTrajectory(
    const std::vector<Eigen::VectorXd>& q_parameters_in)
{
    if (q_parameters_in.size() != T_)
        ThrowPretty("Expected initial trajectory of length "
                    << T_ << " but got " << q_parameters_in.size());
    if (q_parameters_in[0].rows() != N)
        ThrowPretty("Expected states to have " << N << " rows but got "
                                               << q_parameters_in[0].rows());

    initial_trajectory_ = q_parameters_in;
    SetStartState(q_parameters_in[0]);
}

std::vector<Eigen::VectorXd> BoundedTimeIndexedProblem::GetInitialTrajectory()
{
    return initial_trajectory_;
}

double BoundedTimeIndexedProblem::GetDuration()
{
    return tau_ * static_cast<double>(T_);
}

void BoundedTimeIndexedProblem::Update(Eigen::VectorXdRefConst x_in, int t)
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
    }
    else if (flags_ & KIN_J)
    {
        cost.Update(Phi[t], jacobian[t], t);
    }
    else
    {
        cost.Update(Phi[t], t);
    }
    if (t > 0) xdiff[t] = x[t] - x[t - 1];
    ++number_of_problem_updates_;
}

double BoundedTimeIndexedProblem::GetScalarTaskCost(int t)
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

Eigen::VectorXd BoundedTimeIndexedProblem::GetScalarTaskJacobian(int t)
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

double BoundedTimeIndexedProblem::GetScalarTransitionCost(int t)
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

Eigen::VectorXd BoundedTimeIndexedProblem::GetScalarTransitionJacobian(int t)
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

void BoundedTimeIndexedProblem::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
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

void BoundedTimeIndexedProblem::SetRho(const std::string& task_name, const double rho, int t)
{
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

Eigen::VectorXd BoundedTimeIndexedProblem::GetGoal(const std::string& task_name, int t)
{
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            return cost.y[t].data.segment(cost.indexing[i].start, cost.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double BoundedTimeIndexedProblem::GetRho(const std::string& task_name, int t)
{
    for (int i = 0; i < cost.indexing.size(); ++i)
    {
        if (cost.tasks[i]->GetObjectName() == task_name)
        {
            return cost.rho[t](cost.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

void BoundedTimeIndexedProblem::SetT(const int& T_in)
{
    if (T_in <= 2)
    {
        ThrowNamed("Invalid number of timesteps: " << T_in);
    }
    T_ = T_in;
    ReinitializeVariables();
}

void BoundedTimeIndexedProblem::SetTau(const double& tau_in)
{
    if (tau_in <= 0.) ThrowPretty("tau_ is expected to be greater than 0. (tau_=" << tau_in << ")");
    tau_ = tau_in;
    ct = 1.0 / tau_ / T_;
}

void BoundedTimeIndexedProblem::ReinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("BoundedTimeIndexedProblem", "Initialize problem with T=" << T_);

    SetTau(parameters.tau);
    w_scale_ = parameters.Wrate;

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

    N = scene_->GetKinematicTree().GetNumControlledJoints();

    W = Eigen::MatrixXd::Identity(N, N) * w_scale_;
    if (parameters.W.rows() > 0)
    {
        if (parameters.W.rows() == N)
        {
            W.diagonal() = parameters.W * w_scale_;
        }
        else
        {
            ThrowNamed("W dimension mismatch! Expected " << N << ", got " << parameters.W.rows());
        }
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
    ApplyStartState(false);
    PreUpdate();
}
}
