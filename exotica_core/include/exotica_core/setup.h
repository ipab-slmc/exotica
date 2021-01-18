//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#ifndef EXOTICA_CORE_SETUP_H_
#define EXOTICA_CORE_SETUP_H_

#include <memory>
#include <vector>

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/factory.h>
#include <exotica_core/motion_solver.h>
#include <exotica_core/object.h>
#include <exotica_core/planning_problem.h>
#include <exotica_core/property.h>

#include <pluginlib/class_loader.h>

namespace exotica
{
class Setup : public Object, Uncopyable
{
public:
    ~Setup() noexcept
    {
    }

    static std::shared_ptr<Setup> Instance()
    {
        if (!singleton_initialiser_) singleton_initialiser_.reset(new Setup);
        return singleton_initialiser_;
    }

    static void Destroy();

    static void PrintSupportedClasses();
    static std::shared_ptr<exotica::MotionSolver> CreateSolver(const std::string& type, bool prepend = true) { return ToStdPtr(Instance()->solvers_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::shared_ptr<exotica::TaskMap> CreateMap(const std::string& type, bool prepend = true) { return ToStdPtr(Instance()->maps_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::shared_ptr<exotica::PlanningProblem> CreateProblem(const std::string& type, bool prepend = true) { return Instance()->problems_.CreateInstance((prepend ? "exotica/" : "") + type); }
    static std::shared_ptr<exotica::CollisionScene> CreateCollisionScene(const std::string& type, bool prepend = true) { return ToStdPtr(Instance()->collision_scenes_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::shared_ptr<exotica::DynamicsSolver> CreateDynamicsSolver(const std::string& type, bool prepend = true) { return ToStdPtr(Instance()->dynamics_solvers_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::vector<std::string> GetSolvers();
    static std::vector<std::string> GetProblems();
    static std::vector<std::string> GetMaps();
    static std::vector<std::string> GetCollisionScenes();
    static std::vector<std::string> GetDynamicsSolvers();
    static std::vector<Initializer> GetInitializers();

    static std::shared_ptr<exotica::MotionSolver> CreateSolver(const Initializer& init)
    {
        std::shared_ptr<exotica::MotionSolver> ret = ToStdPtr(Instance()->solvers_.createInstance(init.GetName()));
        ret->InstantiateInternal(init);
        return ret;
    }
    static std::shared_ptr<exotica::PlanningProblem> CreateProblem(const Initializer& init)
    {
        std::shared_ptr<exotica::PlanningProblem> ret = Instance()->problems_.CreateInstance(init.GetName());
        ret->InstantiateInternal(init);
        return ret;
    }

    ///
    /// \brief CreateScene instantiate a scene from an initialiser
    ///     The returned scene is independent of the internal EXOTica solver
    ///     or problem state. It can only be used to access the parsed information
    ///     like joint and link names or the kinematics. Changes to the scene
    ///     will not affect the solver or problem.
    /// \param init scene initialiser
    /// \return a shared pointer to the scene
    ///
    static exotica::ScenePtr CreateScene(const Initializer& init)
    {
        exotica::ScenePtr ret = std::make_shared<exotica::Scene>();
        ret->InstantiateInternal(init);
        return ret;
    }

    static std::shared_ptr<exotica::DynamicsSolver> CreateDynamicsSolver(const Initializer& init)
    {
        auto ret = ToStdPtr(Instance()->dynamics_solvers_.createInstance(init.GetName()));
        ret->InstantiateInternal(init);
        return ret;
    }

    static std::shared_ptr<exotica::CollisionScene> CreateCollisionScene(const Initializer& init)
    {
        auto ret = ToStdPtr(Instance()->collision_scenes_.createInstance(init.GetName()));
        ret->InstantiateInternal(init);
        return ret;
    }

private:
    friend PlanningProblem;
    Setup();
    static std::shared_ptr<Setup> singleton_initialiser_;
    // Make sure the singleton does not get copied
    Setup(Setup const&) = delete;
    void operator=(Setup const&) = delete;

    static std::shared_ptr<exotica::TaskMap> CreateMap(const Initializer& init)
    {
        std::shared_ptr<exotica::TaskMap> ret = ToStdPtr(Instance()->maps_.createInstance(init.GetName()));
        ret->InstantiateInternal(init);
        return ret;
    }

    pluginlib::ClassLoader<exotica::MotionSolver> solvers_;
    pluginlib::ClassLoader<exotica::TaskMap> maps_;
    pluginlib::ClassLoader<exotica::CollisionScene> collision_scenes_;
    pluginlib::ClassLoader<exotica::DynamicsSolver> dynamics_solvers_;
    PlanningProblemFac problems_;
};

typedef std::shared_ptr<Setup> SetupPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_SETUP_H_
