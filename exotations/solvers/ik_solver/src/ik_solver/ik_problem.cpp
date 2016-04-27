/*
 *  Created on: 15 Jul 2014
 *      Author: Yiming Yang
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

#include "ik_solver/ik_problem.h"

REGISTER_PROBLEM_TYPE("IKProblem", exotica::IKProblem);
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
namespace exotica
{
  IKProblem::IKProblem()
      : tau_(0.01)
  {

  }

  IKProblem::~IKProblem()
  {
    //TODO
  }

  EReturn IKProblem::reinitialise(rapidjson::Document& document,
      boost::shared_ptr<PlanningProblem> problem)
  {
    clear();
    if (document.IsArray())
    {
      for (rapidjson::SizeType i = 0; i < document.Size(); i++)
      {
        rapidjson::Value& obj = document[i];
        if (obj.IsObject())
        {
          std::string constraintClass;
          if (ok(getJSON(obj["class"], constraintClass)))
          {
            if (knownMaps_.find(constraintClass) != knownMaps_.end())
            {
              TaskMap_ptr taskmap;
              if (ok(
                  TaskMap_fac::Instance().createObject(
                      knownMaps_[constraintClass], taskmap)))
              {
                EReturn ret = taskmap->initialise(obj, server_, scenes_,
                    problem);
                if (ok(ret))
                {
                  if (ret != CANCELLED)
                  {
                    std::string name = taskmap->getObjectName();
                    task_maps_[name] = taskmap;
                    TaskDefinition_ptr task;
                    if (ok(
                        TaskDefinition_fac::Instance().createObject(
                            "TaskSqrError", task)))
                    {
                      TaskSqrError_ptr sqr = boost::static_pointer_cast<
                          TaskSqrError>(task);
                      sqr->setTaskMap(taskmap);
                      int dim;
                      taskmap->taskSpaceDim(dim);
                      sqr->y_star0_.resize(dim);
                      sqr->rho0_(0) = 0.0;
                      sqr->rho1_(0) = 1.0;
                      sqr->object_name_ = name
                          + std::to_string((unsigned long) sqr.get());

                      // TODO: Better implementation of stting goals from JSON
                      sqr->y_star0_.setZero();

                      sqr->setTimeSteps(T_);
                      Eigen::VectorXd tspan(2);
                      Eigen::VectorXi tspani(2);

                      //	TODO fix ndarray problem

                      getJSON(obj["tspan"], tspan);
                      if (tspan(0) <= 0.0) tspan(0) = 0.0;
                      if (tspan(1) >= 1.0) tspan(1) = 1.0;
                      tspani(0) = (int) ((T_ - 1) * tspan(0));
                      tspani(1) = (int) ((T_ - 1) * tspan(1));
                      for (int t = tspani(0); t <= tspani(1); t++)
                      {
                        sqr->registerRho(
                            Eigen::VectorXdRef_ptr(sqr->rho1_.segment(0, 1)),
                            t);
                      }
                      sqr->wasFullyInitialised_ = true;
                      task_defs_[name] = task;
                    }
                    else
                    {
                      INDICATE_FAILURE
                      ;
                      return FAILURE;
                    }
                  }
                  else
                  {
//                    ROS_WARN_STREAM(
//                        "Creation of '"<<constraintClass<<"' cancelled!");
                  }
                }
                else
                {
                  INDICATE_FAILURE
                  ;
                  return FAILURE;
                }
              }
              else
              {
                INDICATE_FAILURE
                ;
                return FAILURE;
              }

            }
            else
            {
//              WARNING("Ignoring unknown constraint '"<<constraintClass<<"'");
            }
          }
          else
          {
            INDICATE_FAILURE
            ;
            return FAILURE;
          }
        }
        else
        {
          INDICATE_FAILURE
          ;
          return FAILURE;
        }
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    return SUCCESS;

  }

  EReturn IKProblem::reinitialise(Problem& msg, boost::shared_ptr<PlanningProblem> problem)
  {
      EReturn ret = PlanningProblem::reinitialise(msg, problem);
      if (!ok(ret))
      {
        INDICATE_FAILURE
        ;
        return ret;
      }
      for(auto def : task_defs_)
      {
          boost::static_pointer_cast<TaskSqrError>(def.second)->setTimeSteps(T_);
          boost::static_pointer_cast<TaskSqrError>(def.second)->wasFullyInitialised_ = true;
      }
      return SUCCESS;
  }

  EReturn IKProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLElement* xmltmp;
    xmltmp = handle.FirstChildElement("W").ToElement();
    if (xmltmp)
    {
      Eigen::VectorXd tmp;
      XML_OK(getVector(*xmltmp, tmp));
      config_w_ = Eigen::MatrixXd::Identity(tmp.rows(), tmp.rows());
      config_w_.diagonal() = tmp;
    }
    xmltmp = handle.FirstChildElement("Tolerance").ToElement();
    if (xmltmp)
    {
      XML_OK(getDouble(*xmltmp, tau_));
    }
    xmltmp = handle.FirstChildElement("T").ToElement();
    if (xmltmp)
    {
      if (ok(getInt(*xmltmp, T_)))
      {
        // Everything is fine
      }
      else
      {
        T_ = 1;
      }
    }
    else
    {
      T_ = 1;
    }
    for (auto& it : task_defs_)
    {
      if (!ok(it.second->setTimeSteps(T_)))
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }
    return SUCCESS;
  }

  int IKProblem::getT()
  {
    return T_;
  }

  Eigen::MatrixXd IKProblem::getW()
  {
    return config_w_;
  }

  double IKProblem::getTau()
  {
    return tau_;
  }

  void IKProblem::setTau(double tau)
  {
    tau_ = tau;
  }
}	//namespace exotica

