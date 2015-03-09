#include "testing_pkg/Exotica/VelocitySolverType_1.h"

VelocitySolverType_1::VelocitySolverType_1(const exotica::OptimisationParameters_t & params) : 
  VelocitySolver(params),
  name ("VelocitySolverType_1")
{
  clearFlags();
}

bool VelocitySolverType_1::getInverse(const Eigen::MatrixXd & big_jacobian, const Eigen::MatrixXd & config_weights, const Eigen::MatrixXd & task_weights, Eigen::MatrixXd & inv_jacobian)
{
  inverse_called = true;
  //!< Basic inverse function
  if (big_jacobian.rows() == big_jacobian.cols())
  {
    inv_jacobian = big_jacobian.inverse();
    return true;
  }
  else
  {
    return false; //!< I cannot solve for non-square matrices
  }
}


bool VelocitySolverType_1::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  if (!derived_element.FirstChildElement("string").ToElement())
  {
    return false;
  }
  else
  {
    string_element = std::string(derived_element.FirstChildElement("string").ToElement()->GetText());
    return true;
  }    
}

REGISTER_VELOCITY_SOLVER_TYPE("VelocitySolverType_1", VelocitySolverType_1);
REGISTER_FOR_TESTING("VelocitySolverType_1", "VelocitySolverTest_default.xml");
