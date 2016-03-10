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

#include "task_definition/TaskSqrError.h"

REGISTER_TASKDEFINITION_TYPE("TaskSqrError", exotica::TaskSqrError);
REGISTER_TASKDEFINITION_TYPE("TaskVelocitySqrError",
    exotica::TaskVelocitySqrError);

namespace exotica
{

  TaskVelocitySqrError::TaskVelocitySqrError()
  {
    order = 1;
    rho0_.resize(1);
    rho1_.resize(1);
    wasFullyInitialised_ = false;
  }

  TaskSqrError::TaskSqrError()
  {
    order = 0;
    rho0_.resize(1);
    rho1_.resize(1);
    wasFullyInitialised_ = false;
  }

  EReturn TaskSqrError::initialiseManual(std::string name, Server_ptr & server,
      boost::shared_ptr<PlanningProblem> prob,
      std::vector<std::pair<std::string,std::string> >& params)
  {
      EReturn ret = TaskDefinition::initialiseManual(name,server,prob,params);
      int dim;
      task_map_->taskSpaceDim(dim);
      y_star0_.resize(dim);
      y_star0_.setZero();
      rho0_(0) = 1.0;
      wasFullyInitialised_ = true;
      return ret;
  }

  EReturn TaskSqrError::initDerived(tinyxml2::XMLHandle & handle)
  {
    //!< Temporaries
    Eigen::VectorXd y_star; //!< The Goal vector
    double rho;
    // Load Rho
    if (handle.FirstChildElement("Rho").ToElement())
    {
      if (ok(getDouble(*(handle.FirstChildElement("Rho").ToElement()), rho)))
      {

        rho0_(0) = rho;
      }
      else
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return PAR_ERR;
    }

    // Load the goal
    if (handle.FirstChildElement("Goal").ToElement())
    {
      if (ok(
          getVector(*(handle.FirstChildElement("Goal").ToElement()), y_star)))
      {
        y_star0_ = y_star;
      }
      else
      {
        int dim;
        getTaskMap()->taskSpaceDim(dim);
        if (dim > 0)
        {
          y_star0_.resize(dim);
          y_star0_.setZero();
        }
        else
        {
          ERROR(
              "Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
          return FAILURE;
        }
      }
    }
    else
    {
      int dim;
      getTaskMap()->taskSpaceDim(dim);
      if (dim > 0)
      {
        y_star0_.resize(dim);
        y_star0_.setZero();
      }
      else
      {
        ERROR(
            "Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
        return FAILURE;
      }
    }

    // Set default number of time steps
    setTimeSteps(1);

    return SUCCESS;
  }

  EReturn TaskSqrError::setTimeSteps(const int T)
  {
    TaskDefinition::setTimeSteps(T);
    y_star_.assign(T, Eigen::VectorXdRef_ptr(y_star0_));
    rho_.assign(T, Eigen::VectorXdRef_ptr(rho0_));
    return SUCCESS;
  }

  EReturn TaskSqrError::registerGoal(Eigen::VectorXdRef_ptr y_star, int t)
  {
    if (wasFullyInitialised_) (*y_star) = (*(y_star_.at(t)));
    y_star_.at(t) = y_star;
    return SUCCESS;
  }

  EReturn TaskSqrError::registerRho(Eigen::VectorXdRef_ptr rho, int t)
  {
    if (wasFullyInitialised_) (*rho) = (*(rho_.at(t)));
    rho_.at(t) = rho;
    return SUCCESS;
  }

  EReturn TaskSqrError::setDefaultGoals(int t)
  {
    if (!wasFullyInitialised_)
    {
      (*(y_star_.at(t))) = y_star0_;
      (*(rho_.at(t))) = rho0_;
    }
    return SUCCESS;
  }

  double TaskSqrError::getRho(int t)
  {
    return (*(rho_.at(t)))(0);
  }

  EReturn TaskSqrError::setRho(int t, double rho)
  {
    if (!rho_.at(t)) return FAILURE;
    (*(rho_.at(t)))(0) = rho;
    return SUCCESS;
  }

  Eigen::VectorXdRef_ptr TaskSqrError::getGoal(int t)
  {
    if (t < y_star_.size())
    {
      return y_star_[t];
    }
    else
    {
      INDICATE_FAILURE
      return y_star_[0];
    }
  }
}
