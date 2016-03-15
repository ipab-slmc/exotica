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

#ifndef EXOTICA_ERROR_CLASS_H
#define EXOTICA_ERROR_CLASS_H

#include "exotica/TaskDefinition.h"//!< The Component base
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Tools.h"        //!< For XML-Parsing/ErrorFunction definition
#include "exotica/Test.h"         //!< For Testing factory
#include <Eigen/Dense>            //!< Generally dense manipulations should be enough
#include <boost/thread/mutex.hpp> //!< The boost thread-library for synchronisation

namespace exotica
{
  class TaskSqrError: public TaskDefinition
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskSqrError();
      virtual ~TaskSqrError()
      {
      }

      virtual EReturn initialiseManual(std::string name, Server_ptr & server,
          boost::shared_ptr<PlanningProblem> prob,
          std::vector<std::pair<std::string,std::string> >& params);

      /**
       * @brief registerGoal Registers a goal reference at time t
       * @param y_star Goal reference
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerGoal(Eigen::VectorXdRef_ptr y_star, int t = 0);

      /**
       * @brief registerGoal Registers rho reference at time t
       * @param y_star Rho reference
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerRho(Eigen::VectorXdRef_ptr rho, int t = 0);

      /**
       * @brief getRho Returns the value of rho at time step t
       * @param t Timestep
       * @return rho
       */
      double getRho(int t);

      /**
       * @brief setRho Returns the value of rho at time step t
       * @param t Timestep
       * @param rho
       */
      EReturn setRho(int t, double rho);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual EReturn setTimeSteps(const int T);
      int getTimeSteps()
      {
        return y_star_.size();
      }

      /**
       * @brief setDefaultGoals Sets Goals and Rhos to default values
       * @return Indicates success
       */
      EReturn setDefaultGoals(int t);

      Eigen::VectorXd y_star0_;    //!< The goal vector
      Eigen::VectorXd rho0_, rho1_;       //!< The scalar inter-task weight
      bool wasFullyInitialised_;

      /**
       * \bref	Get goal
       * @param	t		Time step
       * @return		Goal
       */
      Eigen::VectorXdRef_ptr getGoal(int t = 0);

    protected:
      /**
       * \brief Concrete implementation of the initDerived
       * @param handle  The handle to the XML-element describing the ErrorFunction Function
       * @return        Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

      /** The internal storage **/
      std::vector<Eigen::VectorXdRef_ptr> y_star_;    //!< The goal vector
      std::vector<Eigen::VectorXdRef_ptr> rho_; //!< The scalar inter-task weight

  };
  typedef boost::shared_ptr<TaskSqrError> TaskSqrError_ptr;
  class TaskVelocitySqrError: public TaskSqrError
  {
    public:
      TaskVelocitySqrError();
  };
  typedef boost::shared_ptr<TaskSqrError> TaskSqrError_ptr;
  typedef boost::shared_ptr<TaskVelocitySqrError> TaskVelocitySqrError_ptr;
}
#endif
