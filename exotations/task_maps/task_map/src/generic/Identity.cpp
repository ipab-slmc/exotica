#include "generic/Identity.h"

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);
REGISTER_FOR_XML_TEST("Identity", "Identity.xml");

exotica::Identity::Identity()
{
  //!< Empty constructor
}

exotica::EReturn exotica::Identity::update(const Eigen::VectorXd & x, const int t)
{
  invalidate();
  if (setPhi(x,t) == SUCCESS)
  {
    return setJacobian(Eigen::MatrixXd::Identity(x.size(), x.size()),t);
  }
  else
  {
    return FAILURE;
  }
}

exotica::EReturn exotica::Identity::initDerived(tinyxml2::XMLHandle & handle)
{
  return SUCCESS;
}

exotica::EReturn exotica::Identity::taskSpaceDim(int & task_dim)
{
  task_dim = -1;
  return exotica::SUCCESS;
}
