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

#ifndef EXOTICA_TASK_DEFINITION_H
#define EXOTICA_TASK_DEFINITION_H

#include "exotica/Object.h"       //! The EXOTica base class
#include "exotica/TaskMap.h"      //! The Task map (since we require a ptr to it)
#include "exotica/Tools.h"

#include <Eigen/Dense>
#include <string>
#include <map>

#define REGISTER_TASKDEFINITION_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::TaskDefinition, TYPE, DERIV)

namespace exotica
{
  class TaskDefinition: public Object
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskDefinition();
      virtual ~TaskDefinition()
      {
      }
      ;

      /**
       * \brief Base Initialiser
       * @pre             The TaskMaps must be initialised
       * @post            Will call the initDerived() function if everything is successful
       * @param handle    Handle to the XML Element
       * @param map_list  A map from names to TaskMap pointers (for initialising the map)
       * @return          The result of calling the initDerived() function
       */
      void initBase(tinyxml2::XMLHandle & handle,
          const TaskMap_map & map_list);

      virtual void initialiseManual(std::string name, Server_ptr & server,
          boost::shared_ptr<PlanningProblem> prob,
          std::vector<std::pair<std::string,std::string> >& params);

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
       * \brief Wrapper for the underlying task dimension getter
       * @param  task_dim Task dimension to be returned
       * @return      Indication of success
       */
      void taskSpaceDim(int & task_dim);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual void setTimeSteps(const int T);

      /**
       * \brief Member function for binding the Task definition to a Task-Map
       * @param task_map  Smart pointer to a task-map
       */
      void setTaskMap(const TaskMap_ptr & task_map);

      /**
       * \brief Member function for getting the Task-Map
       * @return          Smart pointer to a task-map
       */
      TaskMap_ptr getTaskMap();

      bool order;

      virtual std::string print(std::string prepend);

    protected:
      /**
       * \brief Initialises members of the derived type: PURE_VIRTUAL
       * @param handle  The handle to the XML-element describing the ErrorFunction Function
       * @return        Should indicate success/failure
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle) = 0;

      boost::shared_ptr<TaskMap> task_map_; //!< Shared pointer to a Task Map from which it gets its inputs
      boost::mutex map_lock_;  //!< Mapping Lock for synchronisation

  };

  typedef Factory<std::string, TaskDefinition> TaskDefinition_fac;
  typedef boost::shared_ptr<TaskDefinition> TaskDefinition_ptr;
  typedef std::map<std::string, TaskDefinition_ptr> TaskDefinition_map;

}
#endif
