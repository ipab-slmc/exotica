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

#ifndef EXOTICA_TASK_TERMINATION_CRITERION_H
#define EXOTICA_TASK_TERMINATION_CRITERION_H

#include "exotica/TaskDefinition.h"//!< The Component base
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Tools.h"        //!< For XML-Parsing/ErrorFunction definition
#include <exotica/TaskTerminationCriterionInitializer.h>

namespace exotica
{
  class TaskTerminationCriterion: public TaskDefinition, public Instantiable<TaskTerminationCriterionInitializer>
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskTerminationCriterion();
      virtual ~TaskTerminationCriterion()
      {
      }

      virtual void Instantiate(TaskTerminationCriterionInitializer& init);

      /**
       * @brief terminate Checks if current state should terminate
       * @param end Returns true if state should terminate
       * @param err Error
       * @return Indication of success
       */
      virtual void terminate(bool & end, double& err, int t = 0);

      /**
       * @brief registerGoal Registers threshold reference at time t
       * @param y_star Threshold reference
       * @param t Time step
       * @return Indication of success
       */
      void registerThreshold(Eigen::VectorXdRef_ptr threshold, int t = 0);

      /**
       * @brief registerGoal Registers a goal reference at time t
       * @param y_star Goal reference
       * @param t Time step
       * @return Indication of success
       */
      void registerGoal(Eigen::VectorXdRef_ptr y_star, int t = 0);

      /**
       * @brief registerGoal Registers rho reference at time t
       * @param y_star Rho reference
       * @param t Time step
       * @return Indication of success
       */
      void registerRho(Eigen::VectorXdRef_ptr rho, int t = 0);

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
      void setRho(int t, double rho);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual void setTimeSteps(const int T);
      int getTimeSteps()
      {
        return y_star_.size();
      }

      /**
       * @brief setDefaultGoals Sets Goals and Rhos to default values
       * @return Indicates success
       */
      void setDefaultGoals(int t);

      /**
       * \bref	Get goal
       * @param	t		Time step
       * @return		Goal
       */
      Eigen::VectorXdRef_ptr getGoal(int t = 0);

      Eigen::VectorXd y_star0_;    //!< The goal vector
      Eigen::VectorXd rho0_, rho1_;       //!< The scalar inter-task weight
      bool wasFullyInitialised_;
      Eigen::VectorXd threshold0_;
    protected:
      /**
       * \brief Derived-Initialisation
       * @param handle XML handle for any derived parameters
       * @return       Should indicate success/failure
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

      /** The internal storage **/
      std::vector<Eigen::VectorXdRef_ptr> y_star_;    //!< The goal vector
      std::vector<Eigen::VectorXdRef_ptr> rho_; //!< The scalar inter-task weight
      std::vector<Eigen::VectorXdRef_ptr> threshold_;
  };

  typedef exotica::Factory<exotica::TaskTerminationCriterion> TerminationCriterionCreator; //!< Convenience name for the EndCriterion Singleton Factory
  typedef boost::shared_ptr<exotica::TaskTerminationCriterion> TaskTerminationCriterion_ptr;
}
#endif
