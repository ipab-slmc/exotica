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

#include "ompl_solver/OMPLsolver.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

REGISTER_MOTIONSOLVER_TYPE("OMPLsolver", exotica::OMPLsolver)

namespace exotica
{
  OMPLsolver::OMPLsolver()
  {

  }

  void OMPLsolver::Instantiate(OMPLsolverInitializer& init)
  {
      parameters_ = init;
      try
      {
        HIGHLIGHT_NAMED(object_name_,"Using ["<<parameters_.Solver<<"]");
        base_solver_ = to_std_ptr(OMPLBaseSolver::base_solver_loader.createInstance("ompl_solver/" + parameters_.Solver));
      } catch (pluginlib::PluginlibException& ex)
      {
        throw_named("EXOTica-OMPL plugin failed to load solver "<<parameters_.Solver<<"!\nError: " << ex.what());
      }
      Initializer baseInit = (Initializer)init;
      base_solver_->initialiseBaseSolver(baseInit);
  }

  OMPLsolver::~OMPLsolver()
  {
    // If this is not empty, your code is bad and you should feel bad!
    // Whoop whoop whoop whoop ...
  }

  std::string OMPLsolver::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Goal:";
    if (prob_) ret += "\n" + prob_->print(prepend + "    ");
    return ret;
  }

  void OMPLsolver::Solve(Eigen::MatrixXd & solution)
  {
    Eigen::VectorXd q0 = prob_->applyStartState();
    setGoalState(prob_->goal_);
    if (base_solver_->solve(q0, solution))
    {
      HIGHLIGHT("OMPL solving succeeded, planning time "<<base_solver_->getPlanningTime()<<"sec");
    }
    else
    {
      throw_solve("OMPL solving failed");
    }
  }

  void OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)

  {
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<SamplingProblem>(pointer);
    base_solver_->specifyProblem(prob_);
  }

  template<typename T> static ompl::base::PlannerPtr allocatePlanner(
      const ob::SpaceInformationPtr &si, const std::string &new_name)
  {
    ompl::base::PlannerPtr planner(new T(si));
    if (!new_name.empty()) planner->setName(new_name);
    planner->setup();
    return planner;
  }

  void OMPLsolver::setGoalState(Eigen::VectorXdRefConst qT, const double eps)
  {
    base_solver_->setGoalState(qT, eps);
  }
} /* namespace exotica */
