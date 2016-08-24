/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include "exotica/TaskDefinition.h"
#include <exotica/TaskDefinitionInitializer.h>

namespace exotica
{
  TaskDefinition::TaskDefinition()
      : order(0)
  {
    //! Empty Constructor...
  }

  TaskMap_ptr TaskDefinition::getTaskMap()
  {
    return task_map_;
  }

  std::string TaskDefinition::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Task Map:";
    ret += "\n" + task_map_->print(prepend + "    ");
    return ret;
  }

  void TaskDefinition::initialiseManual(std::string name, Server_ptr & server,
            boost::shared_ptr<PlanningProblem> prob,
            std::vector<std::pair<std::string,std::string> >& params)
  {
      object_name_ = name + std::to_string((unsigned long) this);
  }

  void TaskDefinition::InstantiateBase(const InitializerGeneric& init_)
  {
      Object::InstatiateObject(init_);
      TaskDefinitionInitializer init(init_);
      TaskMap_map maps;
      getProperty("TaskMaps", init_, maps);
      setTaskMap(maps.at(init.Map.getValue()));
  }

  void TaskDefinition::initBase(tinyxml2::XMLHandle & handle,
      const TaskMap_map & map_list)
  {
    Server_ptr server;
    Object::initBase(handle, server);
    //!< Temporaries

    //!< Attempt to set the task-map
    if (!handle.FirstChildElement("map").ToElement())
    {
      throw_named("Missing map!");
    }
    else
    {
      const char * map_name =
          handle.FirstChildElement("map").ToElement()->Attribute("name");
      if (map_name == nullptr)
      {
        throw_named("Invalid map name!");
      }
      auto it = map_list.find(map_name);
      if (it == map_list.end())
      {
        throw_named("Can't find the map!");
      }
      setTaskMap(it->second);
    }

    initDerived(handle);
  }

  void TaskDefinition::registerPhi(Eigen::VectorXdRef_ptr y, int t)
  {
    LOCK(map_lock_);
    task_map_->registerPhi(y, t);
  }

  void TaskDefinition::registerJacobian(Eigen::MatrixXdRef_ptr J, int t)
  {
    LOCK(map_lock_);
    task_map_->registerJacobian(J, t);
  }

  void TaskDefinition::taskSpaceDim(int & task_dim)
  {
    task_map_->taskSpaceDim(task_dim);
  }

  void TaskDefinition::setTaskMap(
      const boost::shared_ptr<TaskMap> & task_map)
  {
    LOCK(map_lock_);
    task_map_ = task_map;
  }

  void TaskDefinition::setTimeSteps(const int T)
  {
    LOCK(map_lock_);
    if (task_map_ != nullptr)
    {
      return task_map_->setTimeSteps(T);
    }
    else
    {
      throw_named("Invalid map!");
    }

  }
}
