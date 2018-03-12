#ifndef XMLLOADER_H
#define XMLLOADER_H

#include <exotica/MotionSolver.h>
#include <exotica/PlanningProblem.h>
#include <exotica/Property.h>
#include <exotica/Setup.h>

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

    Initializer loadXML(std::string file_name, bool parsePathAsXML = false);
    void loadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false);
    static void load(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false)
    {
        Instance()->loadXML(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    }

    static Initializer load(std::string file_name, bool parsePathAsXML = false)
    {
        return Instance()->loadXML(file_name, parsePathAsXML);
    }

    static std::shared_ptr<exotica::MotionSolver> loadSolver(const std::string& file_name)
    {
        Initializer solver, problem;
        XMLLoader::load(file_name, solver, problem);
        PlanningProblem_ptr any_problem = Setup::createProblem(problem);
        MotionSolver_ptr any_solver = Setup::createSolver(solver);
        any_solver->specifyProblem(any_problem);
        return any_solver;
    }

    static std::shared_ptr<exotica::MotionSolver> loadSolverStandalone(const std::string& file_name)
    {
        Initializer solver = XMLLoader::load(file_name);
        MotionSolver_ptr any_solver = Setup::createSolver(solver);
        return any_solver;
    }

    static std::shared_ptr<exotica::PlanningProblem> loadProblem(const std::string& file_name)
    {
        Initializer problem = XMLLoader::load(file_name);
        PlanningProblem_ptr any_problem = Setup::createProblem(problem);
        return any_problem;
    }

private:
    XMLLoader();
    static std::shared_ptr<XMLLoader> instance_;
};
}

#endif  // XMLLOADER_H
