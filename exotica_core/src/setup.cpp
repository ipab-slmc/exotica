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

#include <type_traits>

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>
#include <exotica_core/server.h>
#include <exotica_core/setup.h>
#include <exotica_core/tools.h>

namespace exotica
{
SetupPtr Setup::singleton_initialiser_ = nullptr;

void Setup::Destroy()
{
    Server::Destroy();
    if (singleton_initialiser_) singleton_initialiser_.reset();
}

void Setup::PrintSupportedClasses()
{
    HIGHLIGHT("Registered solvers:");
    std::vector<std::string> solvers = Instance()->solvers_.getDeclaredClasses();
    for (const std::string& s : solvers)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered problems:");
    std::vector<std::string> problems = Instance()->problems_.GetDeclaredClasses();
    for (const std::string& s : problems)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered task maps:");
    std::vector<std::string> maps = Instance()->maps_.getDeclaredClasses();
    for (const std::string& s : maps)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered collision scenes:");
    std::vector<std::string> scenes = Instance()->collision_scenes_.getDeclaredClasses();
    for (const std::string& s : scenes)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered dynamics solvers:");
    std::vector<std::string> dynamics_solvers = Instance()->dynamics_solvers_.getDeclaredClasses();
    for (const std::string& s : dynamics_solvers)
    {
        HIGHLIGHT(" '" << s << "'");
    }
}

void AppendInitializer(std::shared_ptr<InstantiableBase> it, std::vector<Initializer>& ret)
{
    std::vector<Initializer> temps = it->GetAllTemplates();
    for (Initializer& i : temps)
    {
        bool found = false;
        for (Initializer& j : ret)
        {
            if (i.GetName() == j.GetName())
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            ret.push_back(i);
        }
    }
}

std::vector<Initializer> Setup::GetInitializers()
{
    std::vector<Initializer> ret = Scene().GetAllTemplates();
    std::vector<std::string> solvers = Instance()->solvers_.getDeclaredClasses();
    for (const std::string& s : solvers)
    {
        try
        {
            AppendInitializer(std::static_pointer_cast<InstantiableBase>(CreateSolver(s, false)), ret);
        }
        catch (std::exception& e)
        {
            WARNING_NAMED("Setup::GetInitializers", "Solver '" << s << "' not built. Won't be able to instantiate.");
            continue;
        }
    }
    std::vector<std::string> maps = Instance()->maps_.getDeclaredClasses();
    for (const std::string& s : maps)
    {
        try
        {
            AppendInitializer(std::static_pointer_cast<InstantiableBase>(CreateMap(s, false)), ret);
        }
        catch (std::exception& e)
        {
            WARNING_NAMED("Setup::GetInitializers", "TaskMap '" << s << "' not built. Won't be able to instantiate.");
            continue;
        }
    }
    std::vector<std::string> collision_scenes = Instance()->collision_scenes_.getDeclaredClasses();
    for (const std::string& s : collision_scenes)
    {
        try
        {
            AppendInitializer(std::static_pointer_cast<InstantiableBase>(CreateCollisionScene(s, false)), ret);
        }
        catch (std::exception& e)
        {
            WARNING_NAMED("Setup::GetInitializers", "CollisionScene '" << s << "' not built. Won't be able to instantiate.");
            continue;
        }
    }
    std::vector<std::string> dynamics_solvers = Instance()->dynamics_solvers_.getDeclaredClasses();
    for (const std::string& s : dynamics_solvers)
    {
        try
        {
            AppendInitializer(std::static_pointer_cast<InstantiableBase>(CreateDynamicsSolver(s, false)), ret);
        }
        catch (std::exception& e)
        {
            WARNING_NAMED("Setup::GetInitializers", "DynamicsSolver '" << s << "' not built. Won't be able to instantiate.");
            continue;
        }
    }
    return ret;
}

std::vector<std::string> Setup::GetSolvers() { return Instance()->solvers_.getDeclaredClasses(); }
std::vector<std::string> Setup::GetProblems() { return Instance()->problems_.GetDeclaredClasses(); }
std::vector<std::string> Setup::GetMaps() { return Instance()->maps_.getDeclaredClasses(); }
std::vector<std::string> Setup::GetCollisionScenes() { return Instance()->collision_scenes_.getDeclaredClasses(); }
std::vector<std::string> Setup::GetDynamicsSolvers() { return Instance()->dynamics_solvers_.getDeclaredClasses(); }
Setup::Setup() : solvers_("exotica_core", "exotica::MotionSolver"),
                 maps_("exotica_core", "exotica::TaskMap"),
                 problems_(PlanningProblemFac::Instance()),
                 collision_scenes_("exotica_core", "exotica::CollisionScene"),
                 dynamics_solvers_("exotica_core", "exotica::DynamicsSolver")
{
}
}  // namespace exotica
