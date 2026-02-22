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

#ifndef EXOTICA_CORE_XML_LOADER_H_
#define EXOTICA_CORE_XML_LOADER_H_

#include <exotica_core/motion_solver.h>
#include <exotica_core/planning_problem.h>
#include <exotica_core/property.h>
#include <exotica_core/setup.h>

namespace exotica
{
class XMLLoader
{
public:
    static std::shared_ptr<XMLLoader> Instance()
    {
        if (!instance_) instance_.reset(new XMLLoader);
        return instance_;
    }

    ~XMLLoader() noexcept
    {
    }

    Initializer LoadXML(std::string file_name, bool parsePathAsXML = false);
    void LoadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false);
    static void Load(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false)
    {
        Instance()->LoadXML(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    }

    static Initializer Load(std::string file_name, bool parsePathAsXML = false)
    {
        return Instance()->LoadXML(file_name, parsePathAsXML);
    }

    static std::shared_ptr<exotica::MotionSolver> LoadSolver(const std::string& file_name)
    {
        Initializer solver, problem;
        XMLLoader::Load(file_name, solver, problem);
        PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
        MotionSolverPtr any_solver = Setup::CreateSolver(solver);
        any_solver->SpecifyProblem(any_problem);
        return any_solver;
    }

    static std::shared_ptr<exotica::MotionSolver> LoadSolverStandalone(const std::string& file_name)
    {
        Initializer solver = XMLLoader::Load(file_name);
        MotionSolverPtr any_solver = Setup::CreateSolver(solver);
        return any_solver;
    }

    static std::shared_ptr<exotica::PlanningProblem> LoadProblem(const std::string& file_name)
    {
        Initializer problem = XMLLoader::Load(file_name);
        PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
        return any_problem;
    }

private:
    XMLLoader();
    static std::shared_ptr<XMLLoader> instance_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_XML_LOADER_H_
