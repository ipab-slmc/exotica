#include "testing_pkg/Exotica/PositionSolverType_1.h"

PositionSolverType_1::PositionSolverType_1(
    const exotica::OptimisationParameters_t & params)
    : PositionSolver(params), name("PositionSolverType_1"), params_ref_(
        optimisation_params_), vel_solv_ref_(vel_solver_)
{
  clearFlags();
}

bool PositionSolverType_1::solve(Eigen::VectorXdRefConst init_conf,
    Eigen::VectorXd & solution_vector)
{
  solution_vector = init_conf; //!< Just copy one into the other
  return true;
}

bool PositionSolverType_1::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  if (!derived_element.FirstChildElement("string").ToElement())
  {
    return false;
  }
  else
  {
    string_element = std::string(
        derived_element.FirstChildElement("string").ToElement()->GetText());
    return true;
  }
}

REGISTER_POSITION_SOLVER_TYPE("PositionSolverType_1", PositionSolverType_1);
