#include "testing_pkg/Exotica/TaskType_2.h"
#include <cmath>

ExoticaTaskTest_2::ExoticaTaskTest_2(const exotica::OptimisationParameters_t & optimisation_params) : TaskDefinition(optimisation_params)
{
  name = "ExoticaTask_2";
  int_element = 5;
  string_element = "";  
}

bool ExoticaTaskTest_2::updateTask(Eigen::VectorXdRefConst configuration, int index)
{
  update_called = true;
  if (configuration.size() != 3) { return false; }
  Eigen::VectorXd phi(1);
  phi << std::pow(configuration[0],3) + 2*std::pow(configuration[1],2) - std::pow(configuration[2],4);
  bool success = setPhi(phi, index);
  Eigen::MatrixXd jacobian(1,3);
  jacobian(0,0) = 3*std::pow(configuration[0],2);
  jacobian(0,1) = 4*configuration[1];
  jacobian(0,2) = -4*std::pow(configuration[2],3);
  success &= setJacobian(jacobian, index);
  return success;  //!< Just return true
}

bool ExoticaTaskTest_2::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  if (!derived_element.FirstChildElement("int").ToElement()) { return false; }
  if (!derived_element.FirstChildElement("int").ToElement()->GetText())
  {
    return false;
  }
  else
  {
    int_element = atoi(derived_element.FirstChildElement("int").ToElement()->GetText());
  }
  if (!derived_element.FirstChildElement("string").ToElement()) { return false; }
  if (!derived_element.FirstChildElement("string").ToElement()->GetText())
  {
    return false;
  }
  else
  {
    string_element = std::string(derived_element.FirstChildElement("string").ToElement()->GetText());
  }
  
  return true;
}

REGISTER_TASK_TYPE("ExoticaTask_2", ExoticaTaskTest_2);
REGISTER_FOR_TESTING("ExoticaTask_2", "ExoticaTask2_default.xml");
