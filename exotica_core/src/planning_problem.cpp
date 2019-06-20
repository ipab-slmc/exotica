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

#include <algorithm>
#include <limits>
#include <utility>

#include <exotica_core/planning_problem.h>
#include <exotica_core/server.h>
#include <exotica_core/setup.h>

#include <exotica_core/planning_problem_initializer.h>
#include <exotica_core/task_map_initializer.h>

namespace exotica
{
PlanningProblem::PlanningProblem() = default;
PlanningProblem::~PlanningProblem() = default;

std::string PlanningProblem::Print(const std::string& prepend) const
{
    std::string ret = Object::Print(prepend);
    ret += "\n" + prepend + "  Task definitions:";
    for (const auto& it : task_maps_)
        ret += "\n" + it.second->Print(prepend + "    ");
    return ret;
}

int PlanningProblem::get_num_positions() const
{
    return num_positions_;
}

int PlanningProblem::get_num_velocities() const
{
    return num_velocities_;
}

int PlanningProblem::get_num_controls() const
{
    return num_controls_;
}

Eigen::VectorXd PlanningProblem::ApplyStartState(bool update_traj)
{
    scene_->SetModelState(start_state_.head(num_positions_), t_start, update_traj);
    return scene_->GetControlledState();
}

void PlanningProblem::PreUpdate()
{
    for (auto& it : task_maps_) it.second->PreUpdate();
}

void PlanningProblem::SetStartState(Eigen::VectorXdRefConst x)
{
    const auto num_states = num_positions_ + num_velocities_;
    if (x.rows() == num_states)
    {
        start_state_ = x;
    }
    else if (x.rows() == scene_->GetKinematicTree().GetNumControlledJoints())
    {
        std::vector<std::string> jointNames = scene_->GetControlledJointNames();
        std::vector<std::string> modelNames = scene_->GetModelJointNames();
        for (int i = 0; i < jointNames.size(); ++i)
        {
            for (int j = 0; j < modelNames.size(); ++j)
            {
                if (jointNames[i] == modelNames[j]) start_state_[j] = x(i);
            }
        }
    }
    // TODO: Add support for num_controlled + tangent vector size initialisation
    else
    {
        ThrowNamed("Wrong start state vector size, expected " << num_states << ", got " << x.rows());
    }
}

Eigen::VectorXd PlanningProblem::GetStartState() const
{
    return start_state_;
}

void PlanningProblem::SetStartTime(double t)
{
    t_start = t;
}

double PlanningProblem::GetStartTime() const
{
    return t_start;
}

void PlanningProblem::InstantiateBase(const Initializer& init_in)
{
    Object::InstantiateObject(init_in);
    PlanningProblemInitializer init(init_in);

    task_maps_.clear();
    tasks_.clear();

    // Create the scene
    scene_.reset(new Scene());
    scene_->InstantiateInternal(SceneInitializer(init.PlanningScene));

    // Set size of positions. This is valid for kinematic problems and will be
    // overridden in dynamic problems to set num_velocities_ and num_controls_.
    num_positions_ = scene_->GetKinematicTree().GetNumModelJoints();
    N = scene_->GetKinematicTree().GetNumControlledJoints();

    // Check if dynamics solver has been initialised, and if so update problem
    // properties.
    if (scene_->GetDynamicsSolver() != nullptr)
    {
        // TODO: Strictly speaking N here should correspond to the number of controls, which comes from the dynamic solver - to be fixed!
        num_controls_ = scene_->GetDynamicsSolver()->get_num_controls();

        // TODO: As a current workaround, we assume num_velocities_ = num_positions_
        // This holds true if there are no special joints (including floating base
        // or spherical joints). For sanity, we check that the dynamics solver
        // dimensions match - otherwise throw.
        num_velocities_ = num_positions_;

        if ((scene_->GetDynamicsSolver()->get_num_positions() != scene_->GetDynamicsSolver()->get_num_velocities()) ||
            (scene_->GetDynamicsSolver()->get_num_positions() != num_positions_))
        {
            // The quadrotor and other floating-base robots are currently broken (cf. #571)
            if (scene_->GetDynamicsSolver()->get_num_positions() > num_positions_)
            {
                ThrowPretty("Proper floating-base joints in dynamic problems not yet supported. Cf. #571.");
                // num_positions_ = scene_->GetDynamicsSolver()->get_num_positions();
                // num_velocities_ = scene_->GetDynamicsSolver()->get_num_velocities();
            }
            // If the difference is exactly 1, just add it:
            else if ((scene_->GetDynamicsSolver()->get_num_positions() - scene_->GetDynamicsSolver()->get_num_velocities()) == 1)
            {
                num_velocities_ -= 1;
            }
            // Else, throw for now:
            else
            {
                WARNING_NAMED("PlanningProblem::InstantiateBase",
                              "Size of tangent vector in DynamicsSolver does not satisfy workaround-assumption (cf. #570):\n"
                                  << "num_positions_=" << num_positions_ << " vs (num_positions_=" << scene_->GetDynamicsSolver()->get_num_positions() << ", num_velocities_=" << scene_->GetDynamicsSolver()->get_num_velocities() << ").");
                ThrowPretty("SAD.");
            }
        }
    }

    // Set the start state
    start_state_ = Eigen::VectorXd::Zero(num_positions_ + num_velocities_);
    if (init.StartState.rows() > 0)
        SetStartState(init.StartState);

    // Set the start time
    if (init.StartTime < 0)
        ThrowNamed("Invalid start time " << init.StartTime);
    t_start = init.StartTime;

    // Set the derivative order for Kinematics
    switch (init.DerivativeOrder)
    {
        case 0:
            flags_ = KIN_FK;
            break;
        case 1:
            flags_ = KIN_FK | KIN_J;
            break;
        case 2:
            flags_ = KIN_FK | KIN_J | KIN_J_DOT;
            break;
    }

    KinematicsRequest request;
    request.flags = flags_;

    // Create the maps
    int id = 0;
    for (const Initializer& MapInitializer : init.Maps)
    {
        TaskMapPtr new_map = Setup::CreateMap(MapInitializer);
        new_map->AssignScene(scene_);
        new_map->ns_ = ns_ + "/" + new_map->GetObjectName();
        if (task_maps_.find(new_map->GetObjectName()) != task_maps_.end())
        {
            ThrowNamed("Map '" + new_map->GetObjectName() + "' already exists!");
        }
        std::vector<KinematicFrameRequest> frames = new_map->GetFrames();

        for (size_t i = 0; i < new_map->kinematics.size(); ++i)
            new_map->kinematics[i] = KinematicSolution(id, frames.size());
        id += frames.size();

        request.frames.insert(request.frames.end(), frames.begin(), frames.end());
        task_maps_[new_map->GetObjectName()] = new_map;
        tasks_.push_back(new_map);
    }
    scene_->RequestKinematics(request, std::bind(&PlanningProblem::UpdateTaskKinematics, this, std::placeholders::_1));

    id = 0;
    int idJ = 0;
    for (int i = 0; i < tasks_.size(); ++i)
    {
        tasks_[i]->id = i;
        tasks_[i]->start = id;
        tasks_[i]->length = tasks_[i]->TaskSpaceDim();
        tasks_[i]->start_jacobian = idJ;
        tasks_[i]->length_jacobian = tasks_[i]->TaskSpaceJacobianDim();
        id += tasks_[i]->length;
        idJ += tasks_[i]->length_jacobian;
    }

    if (init.Maps.size() == 0)
    {
        HIGHLIGHT("No maps were defined!");
    }
}

void PlanningProblem::UpdateTaskKinematics(std::shared_ptr<KinematicResponse> response)
{
    for (auto task : tasks_)
        task->kinematics[0].Create(response);
}

void PlanningProblem::UpdateMultipleTaskKinematics(std::vector<std::shared_ptr<KinematicResponse>> responses)
{
    for (auto task : tasks_)
    {
        if (task->kinematics.size() > responses.size())
        {
            ThrowNamed(responses.size() << " responses provided but task " << task->GetObjectName() << " requires " << task->kinematics.size());
        }

        for (size_t i = 0; i < task->kinematics.size(); ++i)
        {
            task->kinematics[i].Create(responses[i]);
        }
    }
}

TaskMapMap& PlanningProblem::GetTaskMaps()
{
    return task_maps_;
}

TaskMapVec& PlanningProblem::GetTasks()
{
    return tasks_;
}

ScenePtr PlanningProblem::GetScene() const
{
    return scene_;
}

std::pair<std::vector<double>, std::vector<double>> PlanningProblem::GetCostEvolution() const
{
    std::pair<std::vector<double>, std::vector<double>> ret;
    for (size_t position = 0; position < cost_evolution_.size(); ++position)
    {
        if (std::isnan(cost_evolution_[position].second)) break;
        double time_point = std::chrono::duration_cast<std::chrono::duration<double>>(cost_evolution_[position].first - cost_evolution_[0].first).count();
        ret.first.push_back(time_point);
        ret.second.push_back(cost_evolution_[position].second);
    }
    return ret;
}

int PlanningProblem::GetNumberOfIterations() const
{
    return static_cast<int>(std::get<0>(GetCostEvolution()).size());
}

double PlanningProblem::GetCostEvolution(int index) const
{
    if (index > -1 && index < cost_evolution_.size())
    {
        return cost_evolution_[index].second;
    }
    else if (index == -1)
    {
        return cost_evolution_[cost_evolution_.size() - 1].second;
    }
    else
    {
        ThrowPretty("Out of range");
    }
}

void PlanningProblem::ResetCostEvolution(size_t size)
{
    cost_evolution_.resize(size);
    cost_evolution_.assign(size, std::make_pair<std::chrono::high_resolution_clock::time_point, double>(std::chrono::high_resolution_clock::now(), std::numeric_limits<double>::quiet_NaN()));
}

void PlanningProblem::SetCostEvolution(int index, double value)
{
    if (index > -1 && index < cost_evolution_.size())
    {
        cost_evolution_[index].first = std::chrono::high_resolution_clock::now();
        cost_evolution_[index].second = value;
    }
    else if (index == -1)
    {
        cost_evolution_[cost_evolution_.size() - 1].first = std::chrono::high_resolution_clock::now();
        cost_evolution_[cost_evolution_.size() - 1].second = value;
    }
    else
    {
        ThrowPretty("Out of range: " << index << " where length=" << cost_evolution_.size());
    }
}
}  // namespace exotica
