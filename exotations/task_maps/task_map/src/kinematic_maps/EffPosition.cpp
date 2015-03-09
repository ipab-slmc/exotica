#include "kinematic_maps/EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);
REGISTER_FOR_XML_TEST("EffPosition", "EffPosition.xml");

exotica::EffPosition::EffPosition()
{
  //!< Empty constructor
}

exotica::EReturn exotica::EffPosition::update(const Eigen::VectorXd & x, const int t)
{
  //!< Prepare
  invalidate();
  LOCK(scene_lock_);
  
  //!< Check
  if (scene_ == nullptr) { INDICATE_FAILURE; return MMB_NIN; }
    
  //!< Temporaries
  std::vector<std::string> temp_vector;
  Eigen::VectorXd phi(scene_->getMapSize()*3);
  Eigen::MatrixXd jac(scene_->getMapSize()*3,x.rows());
  bool success = true;
  EReturn tmp_rtn = FAILURE;
  
  success = scene_->getForwardMap(phi, temp_vector);

  if(!success) { INDICATE_FAILURE; return FAILURE; }
  else { success = scene_->getJacobian(jac); }
  if(!success) { INDICATE_FAILURE; return FAILURE; }
  else { tmp_rtn = setPhi(phi,t); }
  if(!success) { INDICATE_FAILURE; return FAILURE; }
  if (ok(tmp_rtn))  { tmp_rtn = setJacobian(jac,t); }
  
  return tmp_rtn;
}

exotica::EReturn exotica::EffPosition::initDerived(tinyxml2::XMLHandle & handle)
{
  return SUCCESS;
}

exotica::EReturn exotica::EffPosition::taskSpaceDim(int & task_dim)
{
  if(!scene_)
  {
  	task_dim = -1;
  	ERROR("Kinematic scene has not been initialized!");
  	return exotica::MMB_NIN;
  }
  else
  {
  	task_dim = scene_->getMapSize()*3;
  }
  return exotica::SUCCESS;
}
