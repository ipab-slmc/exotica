/*
 *  Created on: 8 Jun 2015
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

#include <frrt/FlexiblePlanner.h>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
namespace ompl
{
  namespace geometric
  {
    FlexiblePlanner::FlexiblePlanner(const base::SpaceInformationPtr &si,
        const std::string & name)
        : base::Planner(si, name), checkCnt_(0)
    {
      specs_.approximateSolutions = true;
      specs_.directed = true;
    }

    FlexiblePlanner::~FlexiblePlanner()
    {

    }

    bool FlexiblePlanner::setUpLocalPlanner(const std::string & xml_file,
        const exotica::Scene_ptr & scene)
    {
      exotica::Initialiser ini;
      exotica::PlanningProblem_ptr prob;
      exotica::MotionSolver_ptr sol;
      ini.initialise(xml_file, ser_, sol, prob, "LocalProblem",
              "FRRTLocal");
      if (sol->type()=="exotica::IKsolver")
      {
        HIGHLIGHT_NAMED("FRRT",
            "Using local planner "<<sol->object_name_<<" at "<<sol.get());
        local_solver_ = boost::static_pointer_cast<exotica::IKsolver>(sol);
      }
      else
      {
        throw_pretty("Unknown solver type!");
      }
      local_solver_->specifyProblem(prob);
      prob->setScene(scene->getPlanningScene());

      if (prob->getTaskDefinitions().find("LocalTask")
          == prob->getTaskDefinitions().end())
      {
        ERROR("Missing XML tag of 'LocalTask'");
        return false;
      }
      local_map_ = boost::static_pointer_cast<exotica::Identity>(
          prob->getTaskMaps().at("CSpaceMap"));
      ser_->getParam(ser_->getName() + "/GlobalTau", gTau_);
      ser_->getParam(ser_->getName() + "/LocalTau", lTau_);

      return true;
    }

    bool FlexiblePlanner::resetSceneAndGoal(const exotica::Scene_ptr & scene,
        const Eigen::VectorXd & goal)
    {
      checkCnt_ = 0;
      global_goal_.setZero(goal.size());
      global_goal_ = goal;
      local_solver_->getProblem()->setScene(scene->getPlanningScene());
      return true;
    }

    bool FlexiblePlanner::localSolve(const Eigen::VectorXd & qs,
        Eigen::VectorXd & qg, Eigen::MatrixXd & solution)
    {

      int dim = (int) si_->getStateDimension();
      bool ret = local_solver_->SolveFullSolution(qs, solution);
      qg.resize(dim);
      qg = solution.row(solution.rows() - 1).transpose();
      checkCnt_ += (solution.rows() - 1);
      return ret;
    }
  }
}

