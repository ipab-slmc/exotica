#include "testing_pkg/Exotica/TaskType_1.h"

ExoticaTaskTest_1::ExoticaTaskTest_1(
    const exotica::OptimisationParameters_t & optimisation_params)
    : TaskDefinition(optimisation_params)
{
  name = "ExoticaTask_1";
  derived_called = false;
  update_called = false;
}

bool ExoticaTaskTest_1::updateTask(Eigen::VectorXdRefConst configuration,
    int index)
{
  update_called = true;
  if (configuration.size() != 3)
  {
    return false;
  }
  Eigen::VectorXd phi(1);
  phi
      << configuration[0] * configuration[0] + 2 * configuration[1]
          + configuration[2];
  bool success = setPhi(phi, index);
  Eigen::MatrixXd jacobian(1, 3);
  jacobian(0, 0) = 2 * configuration[0];
  jacobian(0, 1) = 2;
  jacobian(0, 2) = 1;
  success &= setJacobian(jacobian, index);
  return success;  //!< Just return true
}

bool ExoticaTaskTest_1::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  return true;
}

REGISTER_TASK_TYPE("ExoticaTask_1", ExoticaTaskTest_1);
REGISTER_FOR_TESTING("ExoticaTask_1", "ExoticaTask1_default.xml");
