#include "task_definition/TaskSqrError.h"

REGISTER_TASKDEFINITION_TYPE("TaskSqrError", exotica::TaskSqrError);
REGISTER_TASKDEFINITION_TYPE("TaskVelocitySqrError", exotica::TaskVelocitySqrError);

exotica::TaskSqrError::TaskSqrError()
{
  y_star_ok_ = W_ok_ = rho_ok_ = false;
}

exotica::EReturn exotica::TaskSqrError::initDerived(tinyxml2::XMLHandle & handle)
{
  //!< Temporaries
  Eigen::MatrixXd W;      //!< The weighting matrix
  Eigen::VectorXd y_star; //!< The Goal vector
  double          rho;
  EReturn temp_return = SUCCESS;
  setTimeSteps(1);

  //!< Invalidate everything
  invalidate();
  
  //!< First get the weight matrix, W which is compulsary
  if (!handle.FirstChildElement("Weights").ToElement()) { temp_return = PAR_ERR; }
  if (temp_return) { INDICATE_FAILURE; }
  else             {temp_return = getMatrix(*(handle.FirstChildElement("Weights").ToElement()), W); }
  if (temp_return) { INDICATE_FAILURE; }
  else             { temp_return = setWeight(W); }
  
  //!< Now also get the rho, which is also compulsary
  if (temp_return) { INDICATE_FAILURE; }
  else             { if (!handle.FirstChildElement("Rho").ToElement()) {temp_return = PAR_ERR; } }
  if (temp_return) { INDICATE_FAILURE; }
  else             { temp_return = getDouble(*(handle.FirstChildElement("Rho").ToElement()), rho); }
  if (temp_return) { INDICATE_FAILURE; }
  else             { temp_return = setRho(rho); }
   
  //!< Check if goal specified
  if (temp_return) { INDICATE_FAILURE; }
  else
  {
    if (handle.FirstChildElement("Goal").ToElement())
    {
      temp_return = getVector(*(handle.FirstChildElement("Goal").ToElement()), y_star);
      if (!temp_return) { temp_return = setGoal(y_star); }
      if (temp_return)  { temp_return = WARNING; }  //!< Warning: Goal is not necessary but failure in this case should be warned: otherwise it will be success
    }
    else
    {
      temp_return = WARNING;  //!< Warning that goal is not present
    }
  }
  
  //!< Decide on the return
  if (temp_return and temp_return != WARNING)
  {
    invalidate();
  }
  
  return temp_return;
}


exotica::EReturn exotica::TaskSqrError::setGoal(const Eigen::Ref<const Eigen::VectorXd> & y_star, int t)
{
  LOCK(y_lock_);
  y_star_.at(t) = y_star;
  y_star_ok_ = true;
  return SUCCESS;
}


exotica::EReturn exotica::TaskSqrError::getGoal(Eigen::Ref<Eigen::VectorXd> y_star, int t)
{
  LOCK(y_lock_);
  if (y_star_ok_)
  {
    y_star = y_star_.at(t);
    return SUCCESS;
  }
  else
  {
    INDICATE_FAILURE;
    return MMB_NIN;
  }
}


exotica::EReturn exotica::TaskSqrError::setWeight(const Eigen::Ref<const Eigen::MatrixXd> & W, int t)
{
  LOCK(W_lock_);
  W_.at(t) = W;
  W_ok_ = true;
  return SUCCESS;
}


exotica::EReturn exotica::TaskSqrError::getWeight(Eigen::Ref<Eigen::MatrixXd> W, int t)
{
  LOCK(W_lock_);
  if (W_ok_)
  {
    W = W_.at(t);
    return SUCCESS;
  }
  else
  {
    INDICATE_FAILURE;
    return MMB_NIN;
  }
}


exotica::EReturn exotica::TaskSqrError::setRho(const double & rho, int t)
{
  LOCK(rho_lock_);
  rho_.at(t) = rho;
  rho_ok_ = true;
  return SUCCESS;
}

exotica::EReturn exotica::TaskSqrError::getRho(double & rho, int t)
{
  LOCK(rho_lock_);
  if (rho_ok_)
  {
    rho = rho_.at(t);
    return SUCCESS;
  }
  else
  {
    INDICATE_FAILURE;
    return MMB_NIN;
  }
}
exotica::EReturn exotica::TaskSqrError::modifyGoal(const int & index, const double & value, int t)
{
	//	Modifying goal on the fly, ignore it unless you using DMesh
	LOCK(y_lock_);
	if(!y_star_ok_)
		return FAILURE;
    if(index >= y_star_.at(t).rows())
		return FAILURE;
    y_star_.at(t)(index) = value;
	y_star_ok_ = true;
	return SUCCESS;
}
void exotica::TaskSqrError::invalidate()
{
  LOCK(y_star_lock_);
  LOCK(W_lock_);
  LOCK(rho_lock_);
  
  y_star_ok_ = W_ok_ = rho_ok_ = false;
}

exotica::EReturn exotica::TaskSqrError::setTimeSteps(const int T)
{
    exotica::TaskDefinition::setTimeSteps(T);
    if(y_star_.size()>0)
    {
        y_star_.assign(T,y_star_[0]);
        W_.assign(T,W_[0]);
        rho_.assign(T,rho_[0]);
    }
    else
    {
        y_star_.resize(T);
        W_.resize(T);
        rho_.resize(T);
    }
    return SUCCESS;
}
