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

#include "exotica/PlanningProblem.h"

namespace exotica
{
  PlanningProblem::PlanningProblem()
  {

  }

  void PlanningProblem::reinitialise(rapidjson::Document& document,
      boost::shared_ptr<PlanningProblem> problem)
  {
    throw_named("This has to implemented in the derived class!");
  }

  std::string PlanningProblem::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Task definitions:";
    for (auto& it : task_defs_)
      ret += "\n" + it.second->print(prepend + "    ");
    return ret;
  }

  std::vector<std::pair<std::string, std::string> > vector2map(
      std::vector<std::string> vec)
  {
    std::vector<std::pair<std::string, std::string> > ret;
    int n = vec.size() / 2;
    for (int i = 0; i < n; i++)
    {
      ret.push_back(
          std::pair<std::string, std::string>(vec[i * 2], vec[i * 2 + 1]));
    }
    return ret;
  }

  void PlanningProblem::reinitialise(Problem& msg,
      boost::shared_ptr<PlanningProblem> problem)
  {
    if (type()!=msg.problem_type)
    {
      throw_named("Incorrect problem type!");
    }

    if (msg.map_type.size() < msg.tasks)
    {
      throw_named("Invalid map types!");
    }
    if (msg.task_type.size() < msg.tasks)
    {
      throw_named("Invalid task types!");
    }
    if (msg.task_name.size() < msg.tasks)
    {
      throw_named("Invalid task names!");
    }
    if (msg.task_goal.size() < msg.tasks)
    {
      throw_named("Invalid task goals!");
    }

    for (int i = 0; i < msg.tasks; i++)
    {
      TaskMap_ptr map;
      TaskMap_fac::Instance().createObject(msg.map_type.at(i), map);
      std::vector<std::pair<std::string, std::string> > tmpParams;
      if (msg.map_params.size() != 0)
        tmpParams = vector2map(msg.map_params.at(i).strings);

      map->initialiseManual(msg.task_name.at(i), server_, scenes_, problem,
          tmpParams);
      std::string name = map->getObjectName();
      task_maps_[name] = map;

      TaskDefinition_ptr def;
      TaskDefinition_fac::Instance().createObject(msg.task_type.at(i), def);
      def->setTaskMap(map);
      if (msg.task_params.size() != 0)
        tmpParams = vector2map(msg.task_params.at(i).strings);
      def->initialiseManual(msg.task_name.at(i), server_, problem, tmpParams);
      def->setTimeSteps(msg.task_goal.at(i).row);
      task_defs_[msg.task_name.at(i)] = def;
    }
  }

  void PlanningProblem::initBase(tinyxml2::XMLHandle & handle,
      const Server_ptr & server)
  {
    poses.reset(new std::map<std::string, Eigen::VectorXd>());
    posesJointNames.reset(new std::vector<std::string>());
    knownMaps_["PositionConstraint"] = "Distance";
    knownMaps_["PostureConstraint"] = "Identity";

    startState.resize(0);
    endState.resize(0);
    nominalState.resize(0);

    Object::initBase(handle, server);
    if (!server)
    {
      throw_named("Not fully initialized!");
    }
    server_ = server;
    //!< Temporaries
    tinyxml2::XMLHandle xml_handle(handle);
    int count;
    std::string name;
    std::string type;

    //!< Refresh
    scenes_.clear();
    task_maps_.clear();
    task_defs_.clear();

    //!< First create the Kinematic Scenes
    xml_handle = handle.FirstChildElement("Scene");
    count = 0;
    while (xml_handle.ToElement()) //!< While we are still in a valid situation
    {
      const char * temp_name = xml_handle.ToElement()->Attribute("name");
      if (temp_name == nullptr)
      {
        throw_named("No name specified!");
      }
      name = temp_name;
      if (scenes_.find(name) != scenes_.end())
      {
        throw_named("Can't find the Scene!");
      }
      scenes_[name].reset(new Scene(name));
      if (scenes_[name] == nullptr)
      {
        throw_named("Failed to create a Scene!");
      }
      scenes_.at(name)->initialisation(xml_handle, server_);
      count++;
      xml_handle = xml_handle.NextSiblingElement("Scene");
    }

    //!< No maps defined:
    if (count < 1)
    {
      throw_named("No maps were defined!");
    }

    //!< Now we will create the maps
    xml_handle = handle.FirstChildElement("Map");
    count = 0;
    while (xml_handle.ToElement()) //!< While we are still in a valid situation
    {
      const char * temp_name = xml_handle.ToElement()->Attribute("name");
      if (temp_name == nullptr)
      {
        throw_named("No name specified!");
      }
      name = temp_name;
      if (task_maps_.find(name) != task_maps_.end())
      {
        throw_named("Can't find the map!");
      }
      const char * temp_type = xml_handle.ToElement()->Attribute("type");
      if (temp_type == nullptr)
      {
        throw_named("Can't find the type!");
      }
      type = temp_type;
      TaskMap_ptr temp_ptr;
      TaskMap_fac::Instance().createObject(type, temp_ptr);

        task_maps_[name] = temp_ptr;  //!< Copy the shared_ptr;
        task_maps_.at(name)->ns_ = ns_ + "/" + name;
        count++;
        temp_ptr->initBase(xml_handle, server_, scenes_);
      xml_handle = xml_handle.NextSiblingElement("Map");
    }
    //!< No maps defined:
    if (count < 1)
    {
      HIGHLIGHT("No maps were defined!");
    }

    //!< NEW------------
    //!< Now we initialise the scene
    for (auto & it : scenes_)
    {
      it.second->activateTaskMaps();
    }

    //!< Now the Task Definitions (all)
    xml_handle = handle.FirstChildElement("Task");
    count = 0;
    while (xml_handle.ToElement()) //!< May not be ok due to previous error
    {
      //!< Check that name is available and not duplicated
      const char * temp_name = xml_handle.ToElement()->Attribute("name");
      if (temp_name == nullptr)
      {
        throw_named("Can't find the name!");
      }
      name = temp_name;
      if (task_defs_.find(name) != task_defs_.end())
      {
        throw_named("Can't find the task!");
      }

      //!< Check that Type is also available
      const char * temp_type = xml_handle.ToElement()->Attribute("type");
      if (temp_type == nullptr)
      {
        throw_named("Can't find the type!");
      }
      type = temp_type;

      //!< attempt to create
      TaskDefinition_ptr temp_ptr;
      TaskDefinition_fac::Instance().createObject(type,temp_ptr);

      //!< Attempt to initialise

        task_defs_[name] = temp_ptr;
        task_defs_.at(name)->ns_ = ns_ + "/" + name;
        temp_ptr->initBase(xml_handle, task_maps_);

      //!< Prepare for next iteration (if made it this far)
      count++;
      xml_handle = xml_handle.NextSiblingElement("Task");
    }
    //!< IF no task definitions defined
    if (count < 1)
    {
      HIGHLIGHT("No tasks were defined!");
    }

    //!< If ok so far...
    initDerived(handle);

    originalMaps_ = task_maps_;
    originalDefs_ = task_defs_;
  }

  void PlanningProblem::clear(bool keepOriginals)
  {
    if (keepOriginals)
    {
      task_maps_ = originalMaps_;
      task_defs_ = originalDefs_;
      std::map<std::string,
          std::pair<std::vector<std::string>, std::vector<KDL::Frame> > > tmp;
      for (auto &it : originalMaps_)
      {
        std::pair<std::vector<std::string>, std::vector<KDL::Frame> > tmp_pair;
        try
        {
          it.second->getScene()->getEndEffectors(it.first, tmp_pair);
        }
        catch (Exception e)
        {
          tmp[it.first] = tmp_pair;
        }
      }
      for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
        it->second->clearTaskMap();
      for (auto &it : originalMaps_)
        it.second->getScene()->appendTaskMap(it.first, tmp.at(it.first).first,
            tmp.at(it.first).second);
    }
    else
    {
      task_maps_.clear();
      task_defs_.clear();
      for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
        it->second->clearTaskMap();
    }
  }

  void PlanningProblem::update(Eigen::VectorXdRefConst x, const int t)
  {
    // Update the KinematicScene(s)...
    for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
    {
      it->second->update(x);
    }
    // Update the Task maps

#ifdef EXOTICA_DEBUG_MODE
    if (!((x - x).array() == (x - x).array()).all())
    {
      throw_named("Infinite q= "<<x.transpose());
    }
#endif
    for (TaskMap_map::const_iterator it = task_maps_.begin();
        it != task_maps_.end(); ++it)
    {
      it->second->update(x, t);
    }
  }

  TaskDefinition_map& PlanningProblem::getTaskDefinitions()
  {
    return task_defs_;
  }

  TaskMap_map& PlanningProblem::getTaskMaps()
  {
    return task_maps_;
  }

  Scene_map& PlanningProblem::getScenes()
  {
    return scenes_;
  }

  void PlanningProblem::setScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
    for (auto & it : scenes_)
    {
      it.second->setCollisionScene(scene);
    }
  }

  void PlanningProblem::setScene(
      const moveit_msgs::PlanningSceneConstPtr & scene)
  {
    for (auto & it : scenes_)
    {
      it.second->setCollisionScene(scene);
    }
  }

  void PlanningProblem::updateKinematicScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
    throw_named("Kinematica Scene is duplicated in new EXOTica 3.5, you should not call this function");
  }
}
