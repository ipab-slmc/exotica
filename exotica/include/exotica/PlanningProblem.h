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

#ifndef EXOTICA_MOTION_PLANNING_PROBLEM_H
#define EXOTICA_MOTION_PLANNING_PROBLEM_H

#include "exotica/Object.h"
#include "exotica/TaskMap.h"
#include "exotica/TaskDefinition.h"
#include "exotica/Server.h"
#include "exotica/Scene.h"
#include "exotica/Tools.h"
#include "exotica/Problem.h"
#include "tinyxml2/tinyxml2.h"

#include <vector>
#include <string>
#include <map>

#define REGISTER_PROBLEM_TYPE(TYPE, DERIV) EXOTICA_REGISTER_CORE(exotica::PlanningProblem, TYPE, DERIV)

namespace exotica
{
  class PlanningProblem: public Object, Uncopyable
  {
    public:
      /**
       * \brief Default Constructor
       */
      PlanningProblem();
      virtual ~PlanningProblem()
      {
      }
      ;

      /**
       * \brief Initialiser (from XML): takes care of instantiating the TaskMaps and Definitions and the Kinematic Scenes
       * @param handle[in] The handle to the XML-element describing the Problem Definition
       * @param	server	Server
       * @return           Indication of success/failure: TODO
       */
      void initBase(tinyxml2::XMLHandle & handle, const Server_ptr & server);

      /**
       * \brief Updator: declared virtual so can be overridden.
       * @param x The state of the system
       * @param t Time step (not used by most task maps)
       * @return  Indication of success TODO
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * \brief Returns the reference to the task definition map.
       * @return Task definitions
       */
      TaskDefinition_map& getTaskDefinitions();

      /**
       * \brief	Get task maps
       * @return Task maps
       */
      TaskMap_map& getTaskMaps();

      Scene_map& getScenes();

      /**
       * \brief Update the kinematic scene
       * @param scene	The planning scene from moveit
       */
      void updateKinematicScene(
          const planning_scene::PlanningSceneConstPtr & scene);

      /*
       * \brief	Set EXOTica Scene from a moveit planning scene
       * @param	scene	Moveit planning scene
       */
      void setScene(const planning_scene::PlanningSceneConstPtr & scene);
      void setScene(const moveit_msgs::PlanningSceneConstPtr & scene);
      Scene_map scenes_;  //!< Kinematic scene(s) indexed by name

      Eigen::VectorXd startState;
      Eigen::VectorXd endState;
      Eigen::VectorXd nominalState;
      std::string startStateName;
      std::string endStateName;
      std::string nominalStateName;

      virtual void reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);

      virtual void reinitialise(Problem& msg, boost::shared_ptr<PlanningProblem> problem);

      boost::shared_ptr<std::map<std::string, Eigen::VectorXd> > poses;
      boost::shared_ptr<std::vector<std::string> > posesJointNames;

      virtual std::string print(std::string prepend);

      virtual void clear(bool keepOriginals = true);

      virtual int getT() {throw_named("Not implemented!");}

      virtual double getTau() {throw_named("Not implemented!");}

      virtual void setLimits(Eigen::VectorXd limits) {throw_named("Not implemented!");}

    protected:

      /**
       * \brief Derived Initialiser (from XML): PURE VIRTUAL
       * @param handle[in] The handle to the XML-element describing the Problem Definition
       * @return           Indication of success/failure: TODO
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle) = 0;

      Server_ptr server_; //!< Pointer to EXOTica parameter server;
      TaskMap_map task_maps_; //!< The set of taskmaps we will be using, which will be shared between task-definitions
      TaskDefinition_map task_defs_; //!< The set of task definition objects
      std::map<std::string, std::string> knownMaps_;
      TaskMap_map originalMaps_;
      TaskDefinition_map originalDefs_;

  };

  typedef Factory<PlanningProblem> PlanningProblem_fac;
  typedef boost::shared_ptr<PlanningProblem> PlanningProblem_ptr;

}
#endif
