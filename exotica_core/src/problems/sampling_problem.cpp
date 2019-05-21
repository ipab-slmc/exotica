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

#include <exotica_core/problems/sampling_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("SamplingProblem", exotica::SamplingProblem)

namespace exotica
{
SamplingProblem::SamplingProblem()
{
    flags_ = KIN_FK;
}

SamplingProblem::~SamplingProblem() = default;

std::vector<double> SamplingProblem::GetBounds()
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

void SamplingProblem::Instantiate(const SamplingProblemInitializer& init)
{
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
        ThrowNamed("Dimension mismatch: problem N=" << N << ", but goal state has dimension " << init.Goal.size());
    }

    compound_ = scene_->GetKinematicTree().GetControlledBaseType() != exotica::BaseType::FIXED;

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
    TaskSpaceVector dummy;
    inequality.Initialize(init.Inequality, shared_from_this(), dummy);
    inequality.tolerance = init.ConstraintTolerance;
    equality.Initialize(init.Equality, shared_from_this(), dummy);
    equality.tolerance = init.ConstraintTolerance;
    ApplyStartState(false);

    if (compound_ && init.FloatingBaseLowerLimits.rows() > 0 && init.FloatingBaseUpperLimits.rows() > 0)
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

void SamplingProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    inequality.UpdateS();
    equality.UpdateS();
}

void SamplingProblem::SetGoalState(Eigen::VectorXdRefConst qT)
{
    if (qT.rows() != N)
        ThrowPretty("Dimensionality of goal state wrong: Got " << qT.rows() << ", expected " << N);
    goal_ = qT;
}

void SamplingProblem::SetGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
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

void SamplingProblem::SetRhoEQ(const std::string& task_name, const double& rho)
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

Eigen::VectorXd SamplingProblem::GetGoalEQ(const std::string& task_name)
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

double SamplingProblem::GetRhoEQ(const std::string& task_name)
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

void SamplingProblem::SetGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal)
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

void SamplingProblem::SetRhoNEQ(const std::string& task_name, const double& rho)
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

Eigen::VectorXd SamplingProblem::GetGoalNEQ(const std::string& task_name)
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

double SamplingProblem::GetRhoNEQ(const std::string& task_name)
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

void SamplingProblem::Update(Eigen::VectorXdRefConst x)
{
    scene_->Update(x);
    for (int i = 0; i < num_tasks; ++i)
    {
        if (tasks_[i]->is_used)
            tasks_[i]->Update(x, Phi.data.segment(tasks_[i]->start, tasks_[i]->length));
    }
    inequality.Update(Phi);
    equality.Update(Phi);
    ++number_of_problem_updates_;
}

bool SamplingProblem::IsValid()
{
    // Check bounds
    const Eigen::VectorXd x = scene_->GetKinematicTree().GetControlledState();
    const Eigen::MatrixXd bounds = scene_->GetKinematicTree().GetJointLimits();
    for (int i = 0; i < N; ++i)
    {
        if (x(i) < bounds(i, 0) || x(i) > bounds(i, 1))
        {
            if (debug_) HIGHLIGHT_NAMED("SamplingProblem::IsValid", "State is out of bounds: joint #" << i << ": " << bounds(i, 0) << " < " << x(i) << " < " << bounds(i, 1));
            return false;
        }
    }

    // Check constraints
    const bool inequality_is_valid = ((inequality.S * inequality.ydiff).array() <= 0.0).all();
    const bool equality_is_valid = ((equality.S * equality.ydiff).array().abs() == 0.0).all();

    return (inequality_is_valid && equality_is_valid);
}

bool SamplingProblem::IsValid(Eigen::VectorXdRefConst x)
{
    Update(x);
    return IsValid();
}

int SamplingProblem::GetSpaceDim()
{
    return N;
}

bool SamplingProblem::IsCompoundStateSpace()
{
    return compound_;
}
}
