#include "exotica/TaskDefinition.h"

exotica::TaskDefinition::TaskDefinition()
{
  //! Empty Constructor...
}


exotica::EReturn exotica::TaskDefinition::initBase(tinyxml2::XMLHandle & handle, const TaskMap_map & map_list)
{
    Server_ptr server;
    Object::initBase(handle,server);
  //!< Temporaries
  EReturn tmp_rtn = SUCCESS;
  EReturn aux_rtn = FAILURE;
  
  //!< Attempt to set the task-map
  if(!handle.FirstChildElement("map").ToElement())
  {
    tmp_rtn = WARNING;  //!< Warn if no map set up: this means phi and jacobian will not be available
  }
  else
  {
    const char * map_name = handle.FirstChildElement("map").ToElement()->Attribute("name");
    if (map_name == nullptr)      { INDICATE_FAILURE; return PAR_ERR; }
    auto it = map_list.find(map_name);
    if (it == map_list.end())     { INDICATE_FAILURE; return PAR_ERR; }
    aux_rtn = setTaskMap(it->second);
    if (!ok(aux_rtn))             { INDICATE_FAILURE; return aux_rtn; }
  }
  
  aux_rtn = initDerived(handle);
  if (aux_rtn)
  {
    return aux_rtn;
  }
  else
  {
    return tmp_rtn;
  }
}

exotica::EReturn exotica::TaskDefinition::phi(Eigen::Ref<Eigen::VectorXd> y)
{
    return phi(y,0);
}

exotica::EReturn exotica::TaskDefinition::phi(Eigen::Ref<Eigen::VectorXd> y, int t)
{
  LOCK(map_lock_);
  if (task_map_ != nullptr)
  {
    return task_map_->phi(y,t);
  }
  else
  {
    INDICATE_FAILURE;
    return MEM_ERR;
  }
}

exotica::EReturn exotica::TaskDefinition::jacobian(Eigen::Ref<Eigen::MatrixXd> J)
{
    return jacobian(J,0);
}

exotica::EReturn exotica::TaskDefinition::jacobian(Eigen::Ref<Eigen::MatrixXd> J, int t)
{
  LOCK(map_lock_);
  if (task_map_ != nullptr)
  {
    return task_map_->jacobian(J,t);
  }
  else
  {
    INDICATE_FAILURE;
    return MEM_ERR;
  }
}

exotica::EReturn exotica::TaskDefinition::taskSpaceDim(int & task_dim)
{
	return task_map_->taskSpaceDim(task_dim);
}

exotica::EReturn exotica::TaskDefinition::setTaskMap(const boost::shared_ptr<TaskMap> & task_map)
{
  LOCK(map_lock_);
  task_map_ = task_map;
  return SUCCESS;
}

exotica::EReturn exotica::TaskDefinition::setTimeSteps(const int T)
{
    LOCK(map_lock_);
    if (task_map_ != nullptr)
    {
      return task_map_->setTimeSteps(T);
    }
    else
    {
      INDICATE_FAILURE;
      return MEM_ERR;
    }

}
