/*
 *  Created on: 19 Jun 2014
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

#include <exotica/Problems/SamplingProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("SamplingProblem", exotica::SamplingProblem)
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

namespace exotica
{

  SamplingProblem::SamplingProblem()
      : space_dim_(0), problemType(OMPL_PROBLEM_GOAL)
  {
    // TODO Auto-generated constructor stub

  }

  SamplingProblem::~SamplingProblem()
  {
    // TODO Auto-generated destructor stub
  }

  std::vector<double>& SamplingProblem::getBounds()
  {
    return bounds_;
  }

  void SamplingProblem::clear(bool keepOriginals)
  {
    if (keepOriginals)
    {
      task_maps_ = originalMaps_;
      goals_ = originalGoals_;
    }
    else
    {
      task_maps_.clear();
      task_defs_.clear();
      goals_.clear();
    }
  }

  void SamplingProblem::reinitialise(rapidjson::Document& document,
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
                  TaskMap_ptr taskmap = Setup::createMap(knownMaps_[constraintClass]);
                  taskmap->initialise(obj, server_, scene_,problem);
                  std::string name = taskmap->getObjectName();
                  task_maps_[name] = taskmap;
                  TaskDefinition_ptr task = Setup::createDefinition("TaskTerminationCriterion");
                  TaskTerminationCriterion_ptr sqr =boost::static_pointer_cast<TaskTerminationCriterion>(task);
                  sqr->setTaskMap(taskmap);
                  int dim;
                  taskmap->taskSpaceDim(dim);
                  sqr->y_star0_.resize(dim);
                  sqr->rho0_(0) = 1.0;
                  sqr->threshold0_(0) = 1e-6;
                  sqr->object_name_ = name+ std::to_string((unsigned long) sqr.get());

                  // TODO: Better implementation of stting goals from JSON
                  sqr->y_star0_.setZero();

                  sqr->setTimeSteps(1);
                  sqr->wasFullyInitialised_ = true;
                  task_defs_[name] = task;
                  goals_.push_back(sqr);

            }
            else
            {
//              WARNING("Ignoring unknown constraint '"<<constraintClass<<"'");
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
    std::vector<std::string> jnts;
    scene_->getJointNames(jnts);
    getBounds().resize(jnts.size() * 2);

    if (scene_->getBaseType() == BASE_TYPE::FLOATING
        && server_->hasParam(server_->getName() + "/FloatingBaseLowerLimits")
        && server_->hasParam(server_->getName() + "/FloatingBaseUpperLimits"))
    {

      EParam<exotica::Vector> tmp_lower, tmp_upper;
      server_->getParam(server_->getName() + "/FloatingBaseLowerLimits",tmp_lower);
      server_->getParam(server_->getName() + "/FloatingBaseUpperLimits",tmp_upper);
      if (tmp_lower->data.size() == 6 && tmp_upper->data.size() == 6)
      {
        std::vector<double> lower = tmp_lower->data;
        std::vector<double> upper = tmp_upper->data;
        for (int i = 0; i < 3; i++)
        {
          lower[i] += std::min((double) startState(i), (double) endState(i));
          upper[i] += std::max((double) startState(i), (double) endState(i));
        }
        scene_->getSolver().setFloatingBaseLimitsPosXYZEulerZYX(
            lower, upper);
      }
      else
      {
        throw_named("Can't register parameters!");
      }
    }
    else if (scene_->getBaseType() == BASE_TYPE::FLOATING)
    {
      WARNING("Using floating base without bounds!");
    }

    std::map<std::string, std::vector<double>> joint_limits =
        scene_->getSolver().getUsedJointLimits();
    for (int i = 0; i < 3; i++)
    {
      getBounds()[i] = joint_limits.at(jnts[i])[0];
      getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1];
    }
    for (int i = 6; i < jnts.size(); i++)
    {
      getBounds()[i] = joint_limits.at(jnts[i])[0] - 1e-3;
      getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1] + 1e-3;
    }
  }

  void SamplingProblem::Instantiate(SamplingProblemInitializer& init)
  {
      Parameters = init;
      std::string PlroblemType = init.PlroblemType;
      if(PlroblemType=="Goals")
      {
        for (auto goal : task_defs_)
        {
          if (goal.second->type()=="exotica::TaskTerminationCriterion")
          {
            goals_.push_back( boost::static_pointer_cast<exotica::TaskTerminationCriterion>(goal.second));
          }
          else
          {
            ERROR(goal.first << " has wrong type, ignored!");
          }
        }
       }
      else
      {
          throw_named("Unsupported OMPL problem type!");
      }

      if(init.LocalPlannerConfig!="")
      {
          local_planner_config_ = init.LocalPlannerConfig;
      }

      space_dim_ = scene_->getNumJoints();

      originalMaps_ = task_maps_;
      originalGoals_ = goals_;

      if (scene_->getBaseType() != exotica::BASE_TYPE::FIXED)
        compound_ = true;
      else
        compound_ = false;
      std::vector<std::string> jnts;
      scene_->getJointNames(jnts);

      getBounds().resize(jnts.size() * 2);
      std::map<std::string, std::vector<double>> joint_limits =
          scene_->getSolver().getUsedJointLimits();
      for (int i = 0; i < jnts.size(); i++)
      {
        getBounds()[i] = joint_limits.at(jnts[i])[0];
        getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1];
      }
  }

  void SamplingProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("PlroblemType");
    if (tmp_handle.ToElement())
    {
      std::string tmp = tmp_handle.ToElement()->GetText();
      if (tmp.compare("Costs") == 0)
      {
        problemType = OMPL_PROBLEM_COSTS;
      }
      else if (tmp.compare("GoalBias") == 0)
      {
        problemType = OMPL_PROBLEM_GOAL_BIAS;
      }
      else if (tmp.compare("SamplingBias") == 0)
      {
        problemType = OMPL_PROBLEM_SAMPLING_BIAS;
      }
      else if (tmp.compare("Goals") == 0)
      {
        problemType = OMPL_PROBLEM_GOAL;
      }
      else
      {
        throw_named("Unknown problem type!");
      }
    }
    else
    {
      problemType = OMPL_PROBLEM_GOAL;
    }

    switch (problemType)
    {
    case OMPL_PROBLEM_GOAL:
      for (auto goal : task_defs_)
      {
        if (goal.second->type().compare("exotica::TaskTerminationCriterion")
            == 0)
        {
          goals_.push_back(
              boost::static_pointer_cast<exotica::TaskTerminationCriterion>(
                  goal.second));
        }
        else
        {
          ERROR(goal.first << " has wrong type, ignored!");
        }
      }
      break;
    default:
        throw_named("Unsupported OMPL problem type!");
    }


    tmp_handle = handle.FirstChildElement("LocalPlannerConfig");
    if (tmp_handle.ToElement())
    {
      local_planner_config_ = tmp_handle.ToElement()->GetText();
    }

    space_dim_ = scene_->getNumJoints();

    originalMaps_ = task_maps_;
    originalGoals_ = goals_;

    if (scene_->getBaseType() != exotica::BASE_TYPE::FIXED)
      compound_ = true;
    else
      compound_ = false;
    std::vector<std::string> jnts;
    scene_->getJointNames(jnts);

    getBounds().resize(jnts.size() * 2);
    std::map<std::string, std::vector<double>> joint_limits =
        scene_->getSolver().getUsedJointLimits();
    for (int i = 0; i < jnts.size(); i++)
    {
      getBounds()[i] = joint_limits.at(jnts[i])[0];
      getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1];
    }
  }

  int SamplingProblem::getSpaceDim()
  {
    return space_dim_;
  }

  std::vector<TaskTerminationCriterion_ptr>& SamplingProblem::getGoals()
  {
    return goals_;
  }

  bool SamplingProblem::isCompoundStateSpace()
  {
    return compound_;
  }

} /* namespace exotica */
