#include "exotica/TaskDefinition.h"

namespace exotica
{
    TaskDefinition::TaskDefinition() : order(0)
    {
      //! Empty Constructor...
    }

    TaskMap_ptr TaskDefinition::getTaskMap()
    {
        return task_map_;
    }

    EReturn TaskDefinition::initBase(tinyxml2::XMLHandle & handle, const TaskMap_map & map_list)
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

    EReturn TaskDefinition::registerPhi(Eigen::VectorXdRef_ptr y, int t)
    {
        LOCK(map_lock_);
        y_.at(t)=y;
        task_map_->registerPhi(y,t);
        return SUCCESS;
    }

    EReturn TaskDefinition::registerJacobian(Eigen::MatrixXdRef_ptr J, int t)
    {
        LOCK(map_lock_);
        J_.at(t)=J;
        task_map_->registerJacobian(J,t);
        return SUCCESS;
    }


    EReturn TaskDefinition::taskSpaceDim(int & task_dim)
    {
        return task_map_->taskSpaceDim(task_dim);
    }

    EReturn TaskDefinition::setTaskMap(const boost::shared_ptr<TaskMap> & task_map)
    {
      LOCK(map_lock_);
      task_map_ = task_map;
      return SUCCESS;
    }

    EReturn TaskDefinition::setTimeSteps(const int T)
    {
        LOCK(map_lock_);
        if (task_map_ != nullptr)
        {
          y_.assign(T,Eigen::VectorXdRef_ptr());
          J_.assign(T,Eigen::MatrixXdRef_ptr());
          return task_map_->setTimeSteps(T);
        }
        else
        {
          INDICATE_FAILURE;
          return MEM_ERR;
        }

    }
}
