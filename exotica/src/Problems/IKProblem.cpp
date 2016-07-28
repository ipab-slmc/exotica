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

#include "exotica/Problems/IKProblem.h"
#include "exotica/Initialiser.h"

REGISTER_PROBLEM_TYPE("IKProblem", exotica::IKProblem)

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

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

  void IKProblem::reinitialise(rapidjson::Document& document,
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
                    taskmap->initialise(obj, server_, scenes_,problem);
                    std::string name = taskmap->getObjectName();
                    task_maps_[name] = taskmap;
                    TaskDefinition_ptr task = Initialiser::createDefinition("TaskSqrError");
                    TaskSqrError_ptr sqr = boost::static_pointer_cast<TaskSqrError>(task);
                    sqr->setTaskMap(taskmap);
                    int dim;
                    taskmap->taskSpaceDim(dim);
                    sqr->y_star0_.resize(dim);
                    sqr->rho0_(0) = 0.0;
                    sqr->rho1_(0) = 1.0;
                    sqr->object_name_ = name+ std::to_string((unsigned long) sqr.get());

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
                        sqr->registerRho(Eigen::VectorXdRef_ptr(sqr->rho1_.segment(0, 1)),t);
                    }
                    sqr->wasFullyInitialised_ = true;
                    task_defs_[name] = task;
                }
                else
                {
                    // WARNING("Ignoring unknown constraint '"<<constraintClass<<"'");
                }
            }
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

  void IKProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLElement* xmltmp;
    xmltmp = handle.FirstChildElement("W").ToElement();
    if (xmltmp)
    {
      Eigen::VectorXd tmp;
      getVector(*xmltmp, tmp);
      config_w_ = Eigen::MatrixXd::Identity(tmp.rows(), tmp.rows());
      config_w_.diagonal() = tmp;
    }
    xmltmp = handle.FirstChildElement("Tolerance").ToElement();
    if (xmltmp)
    {
      getDouble(*xmltmp, tau_);
    }
    xmltmp = handle.FirstChildElement("T").ToElement();
    if (xmltmp)
    {
      try
      {
        getInt(*xmltmp, T_);
      }
      catch (Exception e)
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
      it.second->setTimeSteps(T_);
    }
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

