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

#include <exotica_core/problems/time_indexed_sampling_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedSamplingProblem", exotica::TimeIndexedSamplingProblem)

namespace exotica
{
TimeIndexedSamplingProblem::TimeIndexedSamplingProblem()
{
    flags_ = KIN_FK;
}

TimeIndexedSamplingProblem::~TimeIndexedSamplingProblem() = default;

std::vector<double> TimeIndexedSamplingProblem::GetBounds()
{
    std::vector<double> bounds;
    auto joint_limits = scene_->GetKinematicTree().GetJointLimits();

    bounds.resize(2 * N);
    for (unsigned int i = 0; i < N; ++i)
    {
        bounds[i] = joint_limits(i, 0);
        bounds[i + N] = joint_limits(i, 1);
    }

    return bounds;
}

void TimeIndexedSamplingProblem::Instantiate(const TimeIndexedSamplingProblemInitializer& init)
{
    parameters = init;

    if (init.Goal.size() == N)
    {
        goal_ = init.Goal;
    }
    else if (init.Goal.size() == 0)
    {
        goal_ = Eigen::VectorXd::Zero(N);
    }
    else
    {
        ThrowNamed("Dimension mismatch: problem N=" << N << ", but goal state has dimension " << goal_.rows());
    }

    t_goal_ = init.GoalTime;
    if (t_goal_ <= t_start)
        ThrowNamed("Invalid goal time t_goal= " << t_goal_ << ", where t_start=" << t_start);

    if (init.JointVelocityLimits.size() == N)
    {
        vel_limits = init.JointVelocityLimits;
    }
    else if (init.JointVelocityLimits.size() == 1)
    {
        vel_limits = init.JointVelocityLimits(0) * Eigen::VectorXd::Ones(N);
    }
    else
    {
        ThrowNamed("Dimension mismatch: problem N=" << N << ", but joint velocity limits has dimension " << vel_limits.rows());
    }

    num_tasks = tasks_.size();
    length_Phi = 0;
    length_jacobian = 0;
    for (int i = 0; i < num_tasks; ++i)
    {
        AppendVector(Phi.map, tasks_[i]->GetLieGroupIndices());
        length_Phi += tasks_[i]->length;
        length_jacobian += tasks_[i]->length_jacobian;
    }
    Phi.SetZero(length_Phi);

    inequality.Initialize(init.Inequality, shared_from_this(), constraint_phi);
    inequality.tolerance = init.ConstraintTolerance;
    equality.Initialize(init.Equality, shared_from_this(), constraint_phi);
    equality.tolerance = init.ConstraintTolerance;

    ApplyStartState(false);

    if (scene_->GetKinematicTree().GetControlledBaseType() != exotica::BaseType::FIXED && init.FloatingBaseLowerLimits.rows() > 0 && init.FloatingBaseUpperLimits.rows() > 0)
    {
        if (scene_->GetKinematicTree().GetControlledBaseType() == exotica::BaseType::FLOATING && init.FloatingBaseLowerLimits.rows() == 6 && init.FloatingBaseUpperLimits.rows() == 6)
        {
            scene_->GetKinematicTree().SetFloatingBaseLimitsPosXYZEulerZYX(
                std::vector<double>(init.FloatingBaseLowerLimits.data(), init.FloatingBaseLowerLimits.data() + init.FloatingBaseLowerLimits.size()),
                std::vector<double>(init.FloatingBaseUpperLimits.data(), init.FloatingBaseUpperLimits.data() + init.FloatingBaseUpperLimits.size()));
        }
        else if (scene_->GetKinematicTree().GetControlledBaseType() == exotica::BaseType::PLANAR && init.FloatingBaseLowerLimits.rows() == 3 && init.FloatingBaseUpperLimits.rows() == 3)
        {
            scene_->GetKinematicTree().SetPlanarBaseLimitsPosXYEulerZ(
                std::vector<double>(init.FloatingBaseLowerLimits.data(), init.FloatingBaseLowerLimits.data() + init.FloatingBaseLowerLimits.size()),
                std::vector<double>(init.FloatingBaseUpperLimits.data(), init.FloatingBaseUpperLimits.data() + init.FloatingBaseUpperLimits.size()));
        }
        else
        {
            ThrowNamed("Invalid base limits!");
        }
    }

    PreUpdate();
}

double TimeIndexedSamplingProblem::GetGoalTime() const
{
    return t_goal_;
}

void TimeIndexedSamplingProblem::SetGoalTime(const double& t)
{
    t_goal_ = t;
}

Eigen::VectorXd TimeIndexedSamplingProblem::GetGoalState() const
{
    return goal_;
}

void TimeIndexedSamplingProblem::SetGoalState(Eigen::VectorXdRefConst qT)
{
    if (qT.rows() != N)
        ThrowPretty("Dimensionality of goal state wrong: Got " << qT.rows() << ", expected " << N);
    goal_ = qT;
}

void TimeIndexedSamplingProblem::SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != equality.indexing[i].length) ThrowPretty("Expected length of " << equality.indexing[i].length << " and got " << goal.rows());
            equality.y.data.segment(equality.indexing[i].start, equality.indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedSamplingProblem::SetRhoEQ(const std::string& task_name, const double& rho)
{
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            equality.rho(equality.indexing[i].id) = rho;
            PreUpdate();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedSamplingProblem::GetGoalEQ(const std::string& task_name)
{
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            return equality.y.data.segment(equality.indexing[i].start, equality.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedSamplingProblem::GetRhoEQ(const std::string& task_name)
{
    for (int i = 0; i < equality.indexing.size(); ++i)
    {
        if (equality.tasks[i]->GetObjectName() == task_name)
        {
            return equality.rho(equality.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedSamplingProblem::SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != inequality.indexing[i].length) ThrowPretty("Expected length of " << inequality.indexing[i].length << " and got " << goal.rows());
            inequality.y.data.segment(inequality.indexing[i].start, inequality.indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedSamplingProblem::SetRhoNEQ(const std::string& task_name, const double& rho)
{
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            inequality.rho(inequality.indexing[i].id) = rho;
            PreUpdate();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedSamplingProblem::GetGoalNEQ(const std::string& task_name)
{
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            return inequality.y.data.segment(inequality.indexing[i].start, inequality.indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedSamplingProblem::GetRhoNEQ(const std::string& task_name)
{
    for (int i = 0; i < inequality.indexing.size(); ++i)
    {
        if (inequality.tasks[i]->GetObjectName() == task_name)
        {
            return inequality.rho(inequality.indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

bool TimeIndexedSamplingProblem::IsValid(Eigen::VectorXdRefConst x, const double& t)
{
    scene_->Update(x, t);
    for (int i = 0; i < num_tasks; ++i)
    {
        if (tasks_[i]->is_used)
            tasks_[i]->Update(x, Phi.data.segment(tasks_[i]->start, tasks_[i]->length));
    }
    inequality.Update(Phi);
    equality.Update(Phi);
    ++number_of_problem_updates_;

    bool inequality_is_valid = ((inequality.S * inequality.ydiff).array() <= 0.0).all();
    bool equality_is_valid = ((equality.S * equality.ydiff).array().abs() == 0.0).all();

    if (debug_)
    {
        HIGHLIGHT_NAMED("TimeIndexedSamplingProblem::IsValid", "Equality valid? = " << equality_is_valid << "\tInequality valid? = " << inequality_is_valid);
    }

    return (inequality_is_valid && equality_is_valid);
}

void TimeIndexedSamplingProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    inequality.UpdateS();
    equality.UpdateS();
}

void TimeIndexedSamplingProblem::Update(Eigen::VectorXdRefConst x, const double& t)
{
    IsValid(x, t);
    ++number_of_problem_updates_;
}

int TimeIndexedSamplingProblem::GetSpaceDim()
{
    return N;
}
}  // namespace exotica
