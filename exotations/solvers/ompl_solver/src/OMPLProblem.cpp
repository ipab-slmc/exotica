/*
 * OMPLProblem.cpp
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#include "ompl_solver/OMPLProblem.h"
#include "generic/Identity.h"

REGISTER_PROBLEM_TYPE("OMPLProblem", exotica::OMPLProblem);
REGISTER_TASKDEFINITION_TYPE("TaskBias", exotica::TaskBias);

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}

namespace exotica
{

  TaskBias::TaskBias()
      : TaskSqrError()
  {

  }

  OMPLProblem::OMPLProblem()
      : space_dim_(0), problemType(OMPL_PROBLEM_GOAL)
  {
    // TODO Auto-generated constructor stub

  }

  OMPLProblem::~OMPLProblem()
  {
    // TODO Auto-generated destructor stub
  }

  std::vector<double>& OMPLProblem::getBounds()
  {
    return bounds_;
  }

  void OMPLProblem::clear(bool keepOriginals)
  {
    if (keepOriginals)
    {
      task_maps_ = originalMaps_;
      goals_ = originalGoals_;
      costs_ = originalCosts_;
      goalBias_ = originalGoalBias_;
      samplingBias_ = originalSamplingBias_;
    }
    else
    {
      task_maps_.clear();
      task_defs_.clear();
      goals_.clear();
      costs_.clear();
      goalBias_.clear();
      samplingBias_.clear();
    }
  }

  EReturn OMPLProblem::reinitialise(rapidjson::Document& document,
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
              bool IsGoal = false;
              if (knownMaps_[constraintClass].compare("Identity") == 0)
              {
                std::string postureName;
                if (ok(getJSON(obj["postureName"], postureName)))
                {
                  if (postureName.compare("reach_end") != 0)
                  {
                    continue;
                  }
                  else
                  {
                    IsGoal = true;
                  }
                }
                else
                {
                  continue;
                }
              }
              if (problemType == OMPL_PROBLEM_GOAL_BIAS && !IsGoal) continue;
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
                            "TaskTerminationCriterion", task)))
                    {
                      TaskTerminationCriterion_ptr sqr =
                          boost::static_pointer_cast<TaskTerminationCriterion>(
                              task);
                      sqr->setTaskMap(taskmap);
                      int dim;
                      taskmap->taskSpaceDim(dim);
                      sqr->y_star0_.resize(dim);
                      sqr->rho0_(0) = 1.0;
                      sqr->threshold0_(0) = 1e-6;
                      sqr->object_name_ = name
                          + std::to_string((unsigned long) sqr.get());

                      // TODO: Better implementation of stting goals from JSON
                      sqr->y_star0_.setZero();

                      sqr->setTimeSteps(1);
                      sqr->wasFullyInitialised_ = true;
                      task_defs_[name] = task;
                      goals_.push_back(sqr);
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

      // HACK - special case
      // Add goal bias ////////////////////////////
      if (!problem->endStateName.empty()
          && problemType == OMPL_PROBLEM_GOAL_BIAS)
      {
        TaskMap_ptr taskmap;
        if (ok(TaskMap_fac::Instance().createObject("Identity", taskmap)))
        {
          boost::shared_ptr<exotica::Identity> idt = boost::static_pointer_cast<
              exotica::Identity>(taskmap);
          EReturn ret1 = taskmap->initialiseManual("exotica::Identity", server_,
              scenes_, problem);
          EReturn ret = idt->initialise(problem->endStateName,
              *(problem->posesJointNames));
          if (ok(ret) && ok(ret1))
          {
            if (ret != CANCELLED)
            {
              std::string name = taskmap->getObjectName();
              task_maps_[name] = taskmap;
              TaskDefinition_ptr task;
              if (endState.rows() > 0)
              {
                if (ok(
                    TaskDefinition_fac::Instance().createObject("TaskBias",
                        task)))
                {
                  TaskBias_ptr sqr = boost::static_pointer_cast<TaskBias>(task);
                  sqr->setTaskMap(taskmap);
                  int dim;
                  taskmap->taskSpaceDim(dim);
                  sqr->y_star0_.resize(dim);
                  sqr->rho0_(0) = 1.0;
                  sqr->object_name_ = name
                      + std::to_string((unsigned long) sqr.get());
                  sqr->y_star0_.setZero();

                  sqr->setTimeSteps(1);
                  sqr->wasFullyInitialised_ = true;
                  task_defs_[name] = task;
                  goalBias_.push_back(sqr);
                }
                else
                {
                  INDICATE_FAILURE
                  ;
                  return FAILURE;
                }
              }
            }
          }
        }
      }

    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    std::vector<std::string> jnts;
    scenes_.begin()->second->getJointNames(jnts);
    getBounds().resize(jnts.size() * 2);

    if (scenes_.begin()->second->getBaseType() == BASE_TYPE::FLOATING
        && server_->hasParam(server_->getName() + "/FloatingBaseLowerLimits")
        && server_->hasParam(server_->getName() + "/FloatingBaseUpperLimits"))
    {

      EParam<exotica::Vector> tmp_lower, tmp_upper;
      if (ok(
          server_->getParam(server_->getName() + "/FloatingBaseLowerLimits",
              tmp_lower)) && tmp_lower->data.size() == 6
          && ok(
              server_->getParam(server_->getName() + "/FloatingBaseUpperLimits",
                  tmp_upper)) && tmp_upper->data.size() == 6)
      {
        HIGHLIGHT("Setting floating base bounds");
        std::vector<double> lower = tmp_lower->data;
        std::vector<double> upper = tmp_upper->data;
        for (int i = 0; i < 3; i++)
        {
          lower[i] += std::min((double) startState(i), (double) endState(i));
          upper[i] += std::max((double) startState(i), (double) endState(i));
        }
        scenes_.begin()->second->getSolver().setFloatingBaseLimitsPosXYZEulerZYX(
            lower, upper);
      }
      else
      {
        INDICATE_FAILURE
        return FAILURE;
      }
    }
    else if (scenes_.begin()->second->getBaseType() == BASE_TYPE::FLOATING)
    {
      WARNING("Using floating base without bounds!");
    }

    std::map<std::string, std::vector<double>> joint_limits =
        scenes_.begin()->second->getSolver().getUsedJointLimits();
    for (int i = 0; i < jnts.size(); i++)
    {
      getBounds()[i] = -3;//joint_limits.at(jnts[i])[0];
      getBounds()[i + jnts.size()] =3;// joint_limits.at(jnts[i])[1];
    }
    return SUCCESS;

  }

  EReturn OMPLProblem::initDerived(tinyxml2::XMLHandle & handle)
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
        INDICATE_FAILURE
        ;
        return FAILURE;
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

    case OMPL_PROBLEM_COSTS:
      for (auto goal : task_defs_)
      {
        if (goal.second->type().compare("exotica::TaskSqrError") == 0)
        {
          costs_.push_back(
              boost::static_pointer_cast<exotica::TaskSqrError>(goal.second));
        }
        else
        {
          ERROR(goal.first << " has wrong type, ignored!");
        }
      }
      break;

    case OMPL_PROBLEM_GOAL_BIAS:
      for (auto goal : task_defs_)
      {
        if (goal.second->type().compare("exotica::TaskBias") == 0)
        {
          goalBias_.push_back(
              boost::static_pointer_cast<exotica::TaskBias>(goal.second));
        }
        else
        {
          ERROR(goal.first << " has wrong type, ignored!");
        }
      }
      break;

    case OMPL_PROBLEM_SAMPLING_BIAS:
      for (auto goal : task_defs_)
      {
        if (goal.second->type().compare("exotica::TaskBias") == 0)
        {
          samplingBias_.push_back(
              boost::static_pointer_cast<exotica::TaskBias>(goal.second));
        }
        else
        {
          ERROR(goal.first << " has wrong type, ignored!");
        }
      }
      break;
    }

    tmp_handle = handle.FirstChildElement("LocalPlannerConfig");
    if (tmp_handle.ToElement())
    {
      local_planner_config_ = tmp_handle.ToElement()->GetText();
    }

    for (auto scene : scenes_)
    {
      int nn = scene.second->getNumJoints();
      if (space_dim_ == 0)
      {
        space_dim_ = nn;
        continue;
      }
      else
      {
        if (space_dim_ != nn)
        {
          ERROR("Kinematic scenes have different joint space sizes!");
          return FAILURE;
        }
        else
        {
          continue;
        }
      }
    }

    originalMaps_ = task_maps_;
    originalGoals_ = goals_;
    originalCosts_ = costs_;
    originalGoalBias_ = goalBias_;
    originalSamplingBias_ = samplingBias_;

    if (scenes_.begin()->second->getBaseType() != exotica::BASE_TYPE::FIXED)
      compound_ = true;
    else
      compound_ = false;
    std::vector<std::string> jnts;
    scenes_.begin()->second->getJointNames(jnts);

    getBounds().resize(jnts.size() * 2);
    std::map<std::string, std::vector<double>> joint_limits =
        scenes_.begin()->second->getSolver().getUsedJointLimits();
    for (int i = 0; i < jnts.size(); i++)
    {
      getBounds()[i] = joint_limits.at(jnts[i])[0];
      getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1];
    }
    return SUCCESS;
  }

  int OMPLProblem::getSpaceDim()
  {
    return space_dim_;
  }

  std::vector<TaskTerminationCriterion_ptr>& OMPLProblem::getGoals()
  {
    return goals_;
  }

  bool OMPLProblem::isCompoundStateSpace()
  {
    return compound_;
  }
  std::vector<TaskSqrError_ptr>& OMPLProblem::getCosts()
  {
    return costs_;
  }

  std::vector<TaskBias_ptr>& OMPLProblem::getGoalBias()
  {
    return goalBias_;
  }

  std::vector<TaskBias_ptr>& OMPLProblem::getSamplingBias()
  {
    return samplingBias_;
  }

} /* namespace exotica */
