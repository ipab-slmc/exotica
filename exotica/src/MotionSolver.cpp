#include "exotica/MotionSolver.h"

namespace exotica
{

  MotionSolver::MotionSolver()
  {
  }

  EReturn MotionSolver::initBase(tinyxml2::XMLHandle & handle,
      const Server_ptr & server)
  {
    Object::initBase(handle, server);
    if (!server)
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    server_ = server;
    return initDerived(handle);
  }

  EReturn MotionSolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    problem_ = pointer;
    for (auto& map : problem_->getTaskMaps())
    {
      map.second->poses = problem_->poses;
      map.second->posesJointNames = problem_->posesJointNames;
    }
    return SUCCESS;
  }

  std::string MotionSolver::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Problem:";
    if (problem_) ret += "\n" + problem_->print(prepend + "    ");
    return ret;
  }

}
