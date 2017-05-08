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

#ifndef EXOTICA_TASK_MAP_H
#define EXOTICA_TASK_MAP_H

#include "exotica/Object.h"       //!< The EXOTica base class
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Server.h"
#include "exotica/Scene.h"
#include "exotica/Property.h"

#include <Eigen/Dense>            //!< Generally dense manipulations should be enough
#include <boost/thread/mutex.hpp> //!< The boost thread-library for synchronisation
#include <string>

/**
 * \brief Convenience registrar for the TaskMap Type
 */
#define REGISTER_TASKMAP_TYPE(TYPE, DERIV) EXOTICA_REGISTER(exotica::TaskMap, TYPE, DERIV)

#define PHI (*(phi_.at(t)))
#define JAC (*(jac_.at(t)))
#define EFFPHI (*eff_phi_)
#define EFFJAC (*eff_jac_)

namespace exotica
{
  class PlanningProblem;

  class TaskMap: public Object, Uncopyable, public virtual InstantiableBase
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskMap();
      virtual ~TaskMap()
      {
      }

      virtual void InstantiateBase(const Initializer& init);

      /**
       * \brief Initialiser (from XML): mainly resolves the KinematicScene pointer
       * @pre             The Kinematic Scenes must already be initialised
       * @post            If the xml-element contains a 'kscene' tag, then it will attempt to bind the map to that kinematic scene. Otherwise, scene_ will be set to a nullptr.
       * @param handle    The handle to the XML-element describing the task map
       * @param scene_ptr Map of kinematic scenes (Optional: defaulted)
       *                  \n PAR_ERR if could not bind scene information.
       */
      void initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
          const Scene_ptr & scene_ptr);

      /**
       * \brief Updates the output functions (phi and jacobian): PURE VIRTUAL
       * \details The Function should:
       *          \n call invalidate() before starting task-specific execution
       *          \n lock the scene_ pointer (using the scene_lock_ mutex)
       *          \n check that everything it needs (including possibly the scene pointer) is valid
       * @post      Should store the results using setY() and setYDot() if successful
       * @param  x  The State-space vector for the robot
       * @return    Should indicate success/otherwise using the Exotica error types
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t) = 0;

      /**
       * @brief registerPhi Registers a memory location for the output of phi at time t
       * @param y Reference to memory location to be registered
       * @param t Time step
       * @return Indication of success
       */
      void registerPhi(Eigen::VectorXdRef_ptr y, int t);

      /**
       * @brief registerJacobian egisters a memory location for the output of Jacobian at time t
       * @param J Reference to memory location to be registered
       * @param t Time step
       * @return Indication of success
       */
      void registerJacobian(Eigen::MatrixXdRef_ptr J, int t);

      /**
       * \brief Indicator of the Task-Dimension size: PURE VIRTUAL
       * @param task_dim  The dimensionality of the Task space, or -1 if dynamic...
       */
      virtual void taskSpaceDim(int & task_dim) = 0;

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual void setTimeSteps(const int T);

      bool isRegistered(int t);

      Scene_ptr getScene();

      virtual void initialise(const rapidjson::Value& a);
      void initialise(const rapidjson::Value& a, Server_ptr & server,
          const Scene_ptr & scene_ptr, boost::shared_ptr<PlanningProblem> prob);

      bool updateJacobian_;

      std::vector<Eigen::VectorXdRef_ptr> phi_; //!< The Task-space co-ordinates
      std::vector<Eigen::MatrixXdRef_ptr> jac_;    //!< The Task Jacobian matrix

      boost::shared_ptr<std::map<std::string, Eigen::VectorXd> > poses;
      boost::shared_ptr<std::vector<std::string> > posesJointNames;

      virtual std::string print(std::string prepend);

      void registerScene(Scene_ptr scene);
      std::string getSceneName();

      virtual void debug();
    protected:

      /**
       * Member Variables
       */
      Scene_ptr scene_;  //!< The Scene object (smart-pointer):
      boost::mutex scene_lock_;  //!< Synchronisation for the scene object
      Server_ptr server_; //!< Pointer to EXOTica parameter server;
      std::string scene_name_;
      /**
       * \brief Private data members for information hiding and thread-safety
       */

      Eigen::VectorXdRef_ptr eff_phi_; //!< End-effector phi (output of kinematica)
      Eigen::MatrixXdRef_ptr eff_jac_; //!< End-effector Jacobian (output of kinematica)
      virtual bool getEffReferences();
      int phiCnt_;
      int jacCnt_;
      Eigen::VectorXi phiFlag_;
      Eigen::VectorXi jacFlag_;
      Eigen::VectorXd phi0_;
      Eigen::MatrixXd jac0_;

      std::vector<std::string> tmp_eff;
      std::vector<KDL::Frame> tmp_offset;
  };

  //!< Typedefines for some common functionality
  typedef Factory<TaskMap> TaskMap_fac;  //!< Task Map Factory
  typedef boost::shared_ptr<TaskMap> TaskMap_ptr;  //!< Task Map smart pointer
  typedef std::map<std::string, TaskMap_ptr> TaskMap_map; //!< The mapping by name of TaskMaps
}
#endif
