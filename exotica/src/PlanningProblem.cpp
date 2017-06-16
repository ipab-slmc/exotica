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
  PlanningProblem::PlanningProblem() : Flags(KIN_FK)
  {

  }

  std::string PlanningProblem::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Task definitions:";
    for (auto& it : TaskMaps)
      ret += "\n" + it.second->print(prepend + "    ");
    return ret;
  }

  void PlanningProblem::InstantiateBase(const Initializer& init_)
  {
      Object::InstatiateObject(init_);
      PlanningProblemInitializer init(init_);

      TaskMaps.clear();

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
          NewMap->assignScene(scene_);
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
      int i=0;
      id=0;
      for(auto& map : TaskMaps)
      {
          map.second->Kinematics.Create(Response);
          map.second->Id = i;
          map.second->Start = id;
          map.second->Length = map.second->taskSpaceDim();
          i++;
          id += map.second->Length;
      }

      if (init.Maps.size() == 0)
      {
        HIGHLIGHT("No maps were defined!");
      }
  }

  TaskMap_map& PlanningProblem::getTaskMaps()
  {
    return TaskMaps;
  }

  Scene_ptr PlanningProblem::getScene()
  {
    return scene_;
  }

}
