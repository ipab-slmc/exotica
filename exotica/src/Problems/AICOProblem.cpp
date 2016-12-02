/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
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

#include "exotica/Problems/AICOProblem.h"
#include "exotica/Initialiser.h"

REGISTER_PROBLEM_TYPE("AICOProblem", exotica::AICOProblem)

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

namespace exotica
{

    void AICOProblem::reinitialise(rapidjson::Document& document,
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
                getJSON(obj["class"], constraintClass);
                if (knownMaps_.find(constraintClass) != knownMaps_.end())
                {
                    TaskMap_ptr taskmap = Initialiser::createMap(knownMaps_[constraintClass]);
                    taskmap->initialise(obj, server_, scene_, problem);
                    std::string name = taskmap->getObjectName();
                    task_maps_[name] = taskmap;
                    TaskDefinition_ptr task = Initialiser::createDefinition("TaskSqrError");
                    TaskSqrError_ptr sqr = boost::static_pointer_cast<TaskSqrError>(task);
                    sqr->setTaskMap(taskmap);
                    int dim;
                    taskmap->taskSpaceDim(dim);
                    sqr->y_star0_.resize(dim);
                    sqr->rho0_(0) = 0.0;
                    sqr->rho1_(0) = 1e4;
                    sqr->object_name_ = name + std::to_string((unsigned long) sqr.get());

                    // TODO: Better implementation of stting goals from JSON
                    sqr->y_star0_.setZero();

                    sqr->setTimeSteps(T + 2);
                    Eigen::VectorXd tspan(2);
                    Eigen::VectorXi tspani(2);
                    if (obj["tspan"]["__ndarray__"].IsArray())
                    {
                        getJSON(obj["tspan"]["__ndarray__"], tspan);
                    }
                    else
                    {
                        getJSON(obj["tspan"], tspan);
                    }
                    if (tspan(0) <= 0.0) tspan(0) = 0.0;
                    if (tspan(1) >= 1.0) tspan(1) = 1.0;
                    tspani(0) = (int) (T * tspan(0));
                    tspani(1) = (int) (T * tspan(1));
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
                    throw_named("Unknown constraint '"<<constraintClass<<"'");
                }
            }
            else
            {
              throw_named("Invalid JSON document object!");
            }
          }

        }
        else
        {
            throw_named("Invalid JSON array!");
        }
    }

  void AICOProblem::update(Eigen::VectorXdRefConst x, const int t)
  {
    // Update the KinematicScene(s)...
//    for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
//    {
//      it->second->update(x);
//    }
    scene_->update(x);
    // Update task maps if the task definition precision (rho) is non-zero

    for (auto& it : task_defs_)
    {
      boost::shared_ptr<TaskSqrError> task = boost::static_pointer_cast<
          TaskSqrError>(it.second);
      if (task->getRho(t) > 0)
      {
        task->getTaskMap()->update(x, t);
      }
    }
  }

  AICOProblem::AICOProblem()
      : T(0), tau(0), Q_rate(0), W_rate(0), H_rate(0)
  {

  }

  AICOProblem::~AICOProblem()
  {

  }

  void AICOProblem::Instantiate(AICOProblemInitializer& init)
  {
      T = init.T;
      if (T <= 2)
      {
        throw_named("Invalid number of timesteps: "<<T);
      }
      tau = init.Tau;
      Q_rate = init.Qrate;
      H_rate = init.Hrate;
      W_rate = init.Wrate;
      W = Eigen::MatrixXd::Identity(init.W.rows(), init.W.rows());
      W.diagonal() = init.W;

      for (auto& task : task_defs_)
      {
        if (task.second->type()!="TaskSqrError")
          throw_named("Task variable '" + task.first + "'' is not an squared error!");
      }
      // Set number of time steps
      setTime(T);
  }

  void AICOProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLElement* xmltmp;
    bool hastime = false;
    XML_CHECK("T");
    getInt(*xmltmp, T);
    if (T <= 2)
    {
      throw_named("Invalid number of timesteps: "<<T);
    }
    xmltmp = handle.FirstChildElement("duration").ToElement();
    if (xmltmp)
    {
      getDouble(*xmltmp, tau);
      tau = tau / ((double) T);
      hastime = true;
    }
    if (hastime)
    {
      xmltmp = handle.FirstChildElement("tau").ToElement();
      if (xmltmp)
        WARNING("Duration has already been specified, tau is ignored.");
    }
    else
    {
      XML_CHECK("tau");
      getDouble(*xmltmp, tau);
    }

    XML_CHECK("Qrate");
    getDouble(*xmltmp, Q_rate);
    XML_CHECK("Hrate");
    getDouble(*xmltmp, H_rate);
    XML_CHECK("Wrate");
    getDouble(*xmltmp, W_rate);
    {
      Eigen::VectorXd tmp;
      XML_CHECK("W");
      getVector(*xmltmp, tmp);
      W = Eigen::MatrixXd::Identity(tmp.rows(), tmp.rows());
      W.diagonal() = tmp;
    }
    for (TaskDefinition_map::const_iterator it = task_defs_.begin();
        it != task_defs_.end(); ++it)
    {
      if (it->second->type().compare(std::string("TaskSqrError")) == 0)
        ERROR("Task variable " << it->first << " is not an squared error!");
    }
    // Set number of time steps
    setTime(T);
  }

  int AICOProblem::getT()
  {
    return T;
  }

  void AICOProblem::setTime(int T_)
  {
    if (T_ <= 0)
    {
      throw_named("Invalid number of timesteps: "<<T);
    }
    tau = (double) T * tau / (double) T_;
    T = T_;
    // Set number of time steps
    for (auto& it : task_defs_)
    {
      it.second->setTimeSteps(T + 2);
    }
  }

  void AICOProblem::getT(int& T_)
  {
    T_ = T;
  }

  double AICOProblem::getTau()
  {
    return tau;
  }

  void AICOProblem::getTau(double& tau_)
  {
    tau_ = tau;
  }

  double AICOProblem::getDuration()
  {
    return tau * (double) T;
  }

  Eigen::MatrixXd AICOProblem::getW()
  {
    return W;
  }

  double AICOProblem::getQrate()
  {
    return Q_rate;
  }

  double AICOProblem::getWrate()
  {
    return W_rate;
  }

  double AICOProblem::getHrate()
  {
    return H_rate;
  }

} /* namespace exotica */
