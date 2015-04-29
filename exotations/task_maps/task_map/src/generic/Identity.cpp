#include "generic/Identity.h"

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);
REGISTER_FOR_XML_TEST("Identity", "Identity.xml");

exotica::Identity::Identity()
{
  //!< Empty constructor
}

exotica::EReturn exotica::Identity::update(Eigen::VectorXdRefConst x, const int t)
{
  if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
  if(x.rows()==PHI.rows())
  {
    PHI=x;
    if(updateJacobian_)
    {
        JAC=Eigen::MatrixXd::Identity(x.rows(), x.rows());
    }
  }
  else
  {
      INDICATE_FAILURE;
      return FAILURE;
  }
}

exotica::EReturn exotica::Identity::initDerived(tinyxml2::XMLHandle & handle)
{
  return SUCCESS;
}

exotica::EReturn exotica::Identity::taskSpaceDim(int & task_dim)
{
  task_dim = scene_->getNumJoints();
  return exotica::SUCCESS;
}
