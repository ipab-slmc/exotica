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

#include "exotica/Definitions/TaskTerminationCriterion.h"

REGISTER_TASKDEFINITION_TYPE("TaskTerminationCriterion", exotica::TaskTerminationCriterion)

namespace exotica
{

  TaskTerminationCriterion::TaskTerminationCriterion() : threshold_(0.0)
  {
    order = 0;
    rho0_.resize(1);
    rho1_.resize(1);
    threshold0_.resize(1);
    wasFullyInitialised_ = false;
  }

  void TaskTerminationCriterion::Instantiate(TaskTerminationCriterionInitializer& init)
  {
      rho0_(0) = init.Rho;

      if(init.Goal.rows()>0)
      {
          y_star0_ = init.Goal;
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
            throw_named("Task definition '"<<object_name_<<"':Goal was defined not and task map dimension is invalid!");
          }
      }
      threshold0_(0) = (double)init.Threshold;
      setTimeSteps(1);
  }

  void TaskTerminationCriterion::initDerived(tinyxml2::XMLHandle & handle)
  {
      Eigen::VectorXd y_star; //!< The Goal vector
      double rho;
      // Load Rho
      if (handle.FirstChildElement("Rho").ToElement())
      {
        getDouble(*(handle.FirstChildElement("Rho").ToElement()), rho);
        rho0_(0) = rho;
      }
      else
      {
        throw_named("Parameter not found!");
      }

      // Load the goal
      if (handle.FirstChildElement("Goal").ToElement())
      {
        try
        {
          getVector(*(handle.FirstChildElement("Goal").ToElement()), y_star);
          y_star0_ = y_star;
        }
        catch (Exception e)
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
            throw_named("Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
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
          throw_named("Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
        }
      }


      double thr;
      if (handle.FirstChildElement("Threshold").ToElement())
      {
        getDouble(*(handle.FirstChildElement("Threshold").ToElement()),thr);
        threshold0_(0) = thr;
      }
      else
      {
        throw_named("Threshold was not specified");
      }

    setTimeSteps(1);
  }

  void TaskTerminationCriterion::terminate(bool & end, double& err, int t)
  {
    err = ((*(task_map_->phi_.at(t))) - (*(y_star_.at(t)))).squaredNorm()
        * (*(rho_.at(t)))(0);
    end = err <= (*(threshold_.at(t)))(0);
//    	HIGHLIGHT_NAMED(object_name_,"Phi "<<task_map_->phi_.at(t)->transpose()<<" goal "<<y_star_.at(t)->transpose()<<" Err "<<err);
  }

  void TaskTerminationCriterion::registerThreshold(
      Eigen::VectorXdRef_ptr threshold, int t)
  {
    threshold_.at(t) = threshold;
  }

  void TaskTerminationCriterion::setTimeSteps(const int T)
  {
      TaskDefinition::setTimeSteps(T);
      y_star_.assign(T, Eigen::VectorXdRef_ptr(y_star0_));
      rho_.assign(T, Eigen::VectorXdRef_ptr(rho0_));
      threshold_.assign(T, Eigen::VectorXdRef_ptr(threshold0_));
  }


  void TaskTerminationCriterion::registerGoal(Eigen::VectorXdRef_ptr y_star, int t)
  {
    if (wasFullyInitialised_) (*y_star) = (*(y_star_.at(t)));
    y_star_.at(t) = y_star;
  }

  void TaskTerminationCriterion::registerRho(Eigen::VectorXdRef_ptr rho, int t)
  {
    if (wasFullyInitialised_) (*rho) = (*(rho_.at(t)));
    rho_.at(t) = rho;
  }

  void TaskTerminationCriterion::setDefaultGoals(int t)
  {
    if (!wasFullyInitialised_)
    {
      (*(y_star_.at(t))) = y_star0_;
      (*(rho_.at(t))) = rho0_;
    }
  }

  double TaskTerminationCriterion::getRho(int t)
  {
    return (*(rho_.at(t)))(0);
  }

  void TaskTerminationCriterion::setRho(int t, double rho)
  {
    if (!rho_.at(t)) throw_named("Invalid Rho!");
    (*(rho_.at(t)))(0) = rho;
  }

  Eigen::VectorXdRef_ptr TaskTerminationCriterion::getGoal(int t)
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
