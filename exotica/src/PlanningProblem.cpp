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

#include <exotica/PlanningProblem.h>
#include <exotica/Setup.h>
#include <exotica/PlanningProblemInitializer.h>

namespace exotica
{
  PlanningProblem::PlanningProblem() : server_(Server::Instance()), Flags(KIN_FK)
  {

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

  void PlanningProblem::InstantiateBase(const Initializer& init_)
  {
      Object::InstatiateObject(init_);
      PlanningProblemInitializer init(init_);

      // To be removed::
      poses.reset(new std::map<std::string, Eigen::VectorXd>());
      posesJointNames.reset(new std::vector<std::string>());
      knownMaps_["PositionConstraint"] = "Distance";
      knownMaps_["PostureConstraint"] = "Identity";
      startState.resize(0);
      endState.resize(0);
      nominalState.resize(0);

      TaskMaps.clear();
      task_defs_.clear();

      // Create the scene
      scene_.reset(new Scene());
      scene_->InstantiateInternal(SceneInitializer(init.PlanningScene));

      KinematicsRequest Request;
      Request.Flags = Flags;

      // Create the maps
      int id=0;
      for(const Initializer& MapInitializer : init.Maps)
      {
          TaskMap_ptr NewMap = Setup::createMap(MapInitializer);
          NewMap->ns_ = ns_ + "/" + NewMap->getObjectName();
          if (TaskMaps.find(NewMap->getObjectName()) != TaskMaps.end())
          {
              throw_named("Map '"+NewMap->getObjectName()+"' already exists!");
          }
          std::vector<KinematicFrameRequest> frames = NewMap->GetFrames();

          NewMap->Kinematics = KinematicSolution(id, frames.size());
          id += frames.size();
          Request.Frames.insert(Request.Frames.end(), frames.begin(), frames.end());
          TaskMaps[NewMap->getObjectName()] = NewMap;
      }

      std::shared_ptr<KinematicResponse> Response = scene_->RequestKinematics(Request);
      for(auto& map : TaskMaps)
      {
          map.second->Kinematics.Create(Response);
      }

      if (init.Maps.size() == 0)
      {
        HIGHLIGHT("No maps were defined!");
      }

      // Create the task definitions
      for(const Initializer& task : init.Tasks)
      {
          Initializer mapped_task(task);
          mapped_task.addProperty(Property("TaskMaps",true,boost::any(TaskMaps)));
          TaskDefinition_ptr temp_ptr = Setup::createDefinition(mapped_task);
          temp_ptr->ns_ = ns_ + "/" + temp_ptr->getObjectName();
          if (task_defs_.find(temp_ptr->getObjectName()) != task_defs_.end())
          {
              throw_named("Task definition '"+temp_ptr->getObjectName()+"' already exists!");
          }
          task_defs_[temp_ptr->getObjectName()] = temp_ptr;
      }
      if (init.Tasks.size() == 0)
      {
        HIGHLIGHT("No tasks were defined!");
      }

      originalMaps_ = TaskMaps;
      originalDefs_ = task_defs_;
  }

  void PlanningProblem::clear(bool keepOriginals)
  {
    if (keepOriginals)
    {
      TaskMaps = originalMaps_;
      task_defs_ = originalDefs_;
      std::map<std::string,
          std::pair<std::vector<std::string>, std::vector<KDL::Frame> > > tmp;
      for (auto &it : originalMaps_)
      {
        std::pair<std::vector<std::string>, std::vector<KDL::Frame> > tmp_pair;
        try
        {
          //it.second->getScene()->getEndEffectors(it.first, tmp_pair);
        }
        catch (Exception e)
        {
          tmp[it.first] = tmp_pair;
        }
      }
//      for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
//        it->second->clearTaskMap();
      scene_->clearTaskMap();
      //for (auto &it : originalMaps_)
        //it.second->getScene()->appendTaskMap(it.first, tmp.at(it.first).first,
         //   tmp.at(it.first).second);
    }
    else
    {
      TaskMaps.clear();
      task_defs_.clear();
//      for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
//        it->second->clearTaskMap();
      scene_->clearTaskMap();
    }
  }

  void PlanningProblem::update(Eigen::VectorXdRefConst x, const int t)
  {
    // Update the KinematicScene(s)...
//    for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
//    {
//      it->second->update(x);
//    }
    scene_->update(x);
    // Update the Task maps

    for (auto& it : TaskMaps)
    {
      Eigen::VectorXd y(6);
      Eigen::MatrixXd J(6,10);
      it.second->update(x,y,J);
      HIGHLIGHT(y.transpose());
      HIGHLIGHT(J);
    }
  }

  TaskDefinition_map& PlanningProblem::getTaskDefinitions()
  {
    return task_defs_;
  }

  TaskMap_map& PlanningProblem::getTaskMaps()
  {
    return TaskMaps;
  }

  Scene_ptr PlanningProblem::getScene()
  {
    return scene_;
  }

  void PlanningProblem::setScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
//    for (auto & it : scenes_)
//    {
//      it.second->setCollisionScene(scene);
//    }
    scene_->setCollisionScene(scene);
  }

  void PlanningProblem::setScene(
      const moveit_msgs::PlanningSceneConstPtr & scene)
  {
//    for (auto & it : scenes_)
//    {
//      it.second->setCollisionScene(scene);
//    }
    scene_->setCollisionScene(scene);
  }

  void PlanningProblem::updateKinematicScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
    throw_named("Kinematica Scene is duplicated in new EXOTica 3.5, you should not call this function");
  }
}
