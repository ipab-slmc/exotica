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
#include <pluginlib/class_loader.h>
REGISTER_MOTIONSOLVER_TYPE("OMPLsolver", exotica::OMPLsolver);

namespace exotica
{
  OMPLsolver::OMPLsolver()
  {

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

  void OMPLsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    if (base_solver_->solve(q0, solution))
    {
      HIGHLIGHT(
          "OMPL solving succeeded, planning time "<<base_solver_->getPlanningTime()<<"sec");
    }
    else
    {
      throw_solve("OMPL solving failed");
    }
  }

  void OMPLsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement(
        "TrajectorySmooth");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, smooth_);

    tmp_handle = handle.FirstChildElement("Solver");
    server_->registerParam<std_msgs::String>(ns_, tmp_handle, solver_);

    tmp_handle = handle.FirstChildElement("SolverPackage");
    server_->registerParam<std_msgs::String>(ns_, tmp_handle,solver_package_);

    try
    {
      pluginlib::ClassLoader<exotica::OMPLBaseSolver> base_solver_loader(
          solver_package_->data, "exotica::OMPLBaseSolver");
      base_solver_ = base_solver_loader.createInstance(
          "exotica::" + solver_->data);
      HIGHLIGHT_NAMED(object_name_,
          "Using ["<<solver_->data<<"] from package ["<<solver_package_->data<<"].");
    } catch (pluginlib::PluginlibException& ex)
    {
      throw_named("EXOTica-OMPL plugin failed to load solver "<<solver_->data<<". Solver package: '"<<solver_package_->data<< "'. \nError: " << ex.what());
    }
    base_solver_->initialiseBaseSolver(handle, server_);
  }

  void OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)

  {
    MotionSolver::specifyProblem(pointer);
    prob_ = boost::static_pointer_cast<OMPLProblem>(pointer);

    for (auto & it : prob_->getScenes())
    {
      it.second->activateTaskMaps();
    }
    base_solver_->specifyProblem(prob_);
  }

  bool OMPLsolver::isSolvable(const PlanningProblem_ptr & prob)
  {
    if (prob->type().compare("exotica::OMPLProblem") == 0) return true;
    return false;
  }

  template<typename T> static ompl::base::PlannerPtr allocatePlanner(
      const ob::SpaceInformationPtr &si, const std::string &new_name)
  {
    ompl::base::PlannerPtr planner(new T(si));
    if (!new_name.empty()) planner->setName(new_name);
    planner->setup();
    return planner;
  }

  void OMPLsolver::setGoalState(const Eigen::VectorXd & qT, const double eps)
  {
    base_solver_->setGoalState(qT, eps);
  }
} /* namespace exotica */
