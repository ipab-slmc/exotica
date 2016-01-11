/*
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

#include "exotica_moveit/EXOTicaPlannerService.h"

namespace exotica
{
  EXOTicaPlannerService::EXOTicaPlannerService()
      : as_(nh_, "/ExoticaPlanning",
          boost::bind(&exotica::EXOTicaPlannerService::solve, this, _1), false), initialised_(
          false)
  {

  }

  EXOTicaPlannerService::~EXOTicaPlannerService()
  {

  }

  bool EXOTicaPlannerService::initialise(const std::string & config,
      const std::string & solver, const std::string & problem,
      const std::string & group)
  {
    exotica::Initialiser ini;

    std::vector<exotica::MotionSolver_ptr> solvers;
    std::vector<exotica::PlanningProblem_ptr> probs;
    std::vector<std::string> solver_names(1), prob_names(2);
    solver_names[0] = solver;
    prob_names[0] = problem;
    prob_names[1] = problem + "Bias";
    INFO("Loading from "<<config);
    initialised_ = true;
    if (!ok(
        ini.initialise(config, server_, solvers, probs, prob_names,
            solver_names)))
    {
      ERROR("EXOTica/MoveIt Action service: EXOTica initialisation failed !!!!");
      initialised_ = false;
    }
    else
    {
      solver_ = solvers[0];
      problem_ = probs[0];
      if (solver_->type().compare("exotica::OMPLsolver") == 0)
      {
        if (problem_->getTaskMaps().find("CSpaceGoalMap")
            == problem_->getTaskMaps().end()
            || problem_->getTaskMaps().at("CSpaceGoalMap")->type().compare(
                "exotica::Identity") != 0)
        {
          INDICATE_FAILURE
          initialised_ = false;
        }
        else
        {
          goal_map_ = boost::static_pointer_cast<exotica::Identity>(
              problem_->getTaskMaps().at("CSpaceGoalMap"));
        }

        if (probs[1]->getTaskMaps().find("GoalBiasMap")
            == probs[1]->getTaskMaps().end()
            || probs[1]->getTaskMaps().at("GoalBiasMap")->type().compare(
                "exotica::Identity") != 0)
        {
          INDICATE_FAILURE
          initialised_ = false;
        }
        else
        {
          goal_bias_map_ = boost::static_pointer_cast<exotica::Identity>(
              probs[1]->getTaskMaps().at("GoalBiasMap"));
        }

        const moveit::core::JointModelGroup* model_group = server_->getModel(
            "robot_description")->getJointModelGroup(group);
        moveit::core::JointBoundsVector b =
            model_group->getActiveJointModelsBounds();
//				exotica::OMPLProblem_ptr tmp =
//						boost::static_pointer_cast<exotica::OMPLProblem>(problem_);
//				tmp->getBounds().resize(b.size() * 2);
//				for (int i = 0; i < b.size(); i++)
//				{
//					tmp->getBounds()[i] = (*b[i])[0].min_position_;
//					tmp->getBounds()[i + b.size()] = (*b[i])[0].max_position_;
//				}
        if (!exotica::ok(
            boost::static_pointer_cast<exotica::OMPLsolver>(solver_)->specifyProblem(
                probs[0], NULL, probs[1], NULL)))
        {
          INDICATE_FAILURE
          initialised_ = false;
        }
      }
      else if (!exotica::ok(solver_->specifyProblem(problem_)))
      {
        INDICATE_FAILURE
        initialised_ = false;
      }

      if (initialised_) as_.start();
    }
    return initialised_;
  }
  bool EXOTicaPlannerService::solve(
      const exotica_moveit::ExoticaPlanningGoalConstPtr & goal)
  {
    res_.succeeded_ = false;
    scene_.reset(new moveit_msgs::PlanningScene(goal->scene_));
    if (!ok(problem_->setScene(scene_)))
    {
      INDICATE_FAILURE
      return false;
    }
//		HIGHLIGHT_NAMED("MoveitInterface", "Using Solver "<<solver_->object_name_<<"["<<solver_->type()<<"], Problem "<<problem_->object_name_<<"["<<problem_->type()<<"].");

    int benchmark_cnt = 1;
    for (int i = 0; i < benchmark_cnt; i++)
    {
      if (solver_->type().compare("exotica::OMPLsolver") == 0)
      {
        exotica::OMPLsolver_ptr ss = boost::static_pointer_cast<
            exotica::OMPLsolver>(solver_);

        Eigen::VectorXd qT;
        exotica::vectorExoticaToEigen(goal->qT, qT);
        int dim = 0;
        goal_bias_map_->taskSpaceDim(dim);
        goal_bias_map_->jointRef = qT;
        goal_map_->jointRef = qT;
        ss->setGoalState(qT, 1e-4);
        ss->setMaxPlanningTime(goal->max_time_);
        if (!ok(ss->resetIfNeeded()))
        {
          INDICATE_FAILURE
          return FAILURE;
        }
      }

      EReturn found = FAILURE;
      Eigen::VectorXd q0;
      Eigen::MatrixXd solution;
      exotica::vectorExoticaToEigen(goal->q0, q0);
      ros::Time start = ros::Time::now();
      if (solver_->type().compare("exotica::IKsolver") == 0)
      {
        found =
            boost::static_pointer_cast<exotica::IKsolver>(solver_)->SolveFullSolution(
                q0, solution);
      }
      else
        found = solver_->Solve(q0, solution);
      if (ok(found))
      {
        res_.succeeded_ = true;
        fb_.solving_time_ = res_.planning_time_ = ros::Duration(
            ros::Time::now() - start).toSec();
        exotica::matrixEigenToExotica(solution, res_.solution_);
      }
      else
        res_.succeeded_ = false;
      as_.setSucceeded(res_);
      if (benchmark_cnt > 1)
        HIGHLIGHT_NAMED("OMPL Benchmark",
            "Solving time "<<i<<"/ "<<benchmark_cnt<< (ok(found) ? "Succeed":"Failed"));
    }
    return res_.succeeded_;
  }
}

