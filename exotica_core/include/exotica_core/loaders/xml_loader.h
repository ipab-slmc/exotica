#ifndef XMLLOADER_H
#define XMLLOADER_H

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
}

#endif  // XMLLOADER_H
