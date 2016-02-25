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

  EReturn OMPLsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    EReturn ret = base_solver_->solve(q0, solution);
    if (ok(ret))
    {
      HIGHLIGHT(
          "OMPL solving succeeded, planning time "<<base_solver_->getPlanningTime()<<"sec");
    }
    else
    {
      HIGHLIGHT("OMPL solving failed");
    }
    return ret;
  }

//  EReturn OMPLsolver::convertPath(const ompl::geometric::PathGeometric &pg,
//      Eigen::MatrixXd & traj)
//  {
//    traj.resize(pg.getStateCount(), state_space_->getDimension());
//    Eigen::VectorXd tmp(state_space_->getDimension());
//
//    for (int i = 0; i < (int) pg.getStateCount(); ++i)
//    {
//      if (!ok(
//          compound_ ?
//              boost::static_pointer_cast<OMPLSE3RNCompoundStateSpace>(
//                  state_space_)->OMPLStateToEigen(pg.getState(i), tmp) :
//              boost::static_pointer_cast<OMPLStateSpace>(state_space_)->copyFromOMPLState(
//                  pg.getState(i), tmp)))
//      {
//        ERROR("Can't copy state "<<i);
//        return FAILURE;
//      }
//      else
//      {
//        traj.row(i) = tmp;
//      }
//    }
//    return SUCCESS;
//  }
//
//  EReturn OMPLsolver::getSimplifiedPath(ompl::geometric::PathGeometric &pg,
//      Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc)
//  {
//    if (smooth_->data)
//    {
//      HIGHLIGHT("Simplifying solution");
//      int original_cnt = pg.getStateCount();
//      ros::Time start = ros::Time::now();
//
//      //ompl_simple_setup_->simplifySolution(d);
//      // Lets do our own simplifier ~:)
//      if (original_cnt >= 3)
//      {
//        og::PathSimplifierPtr psf_ = ompl_simple_setup_->getPathSimplifier();
//        const ob::SpaceInformationPtr &si =
//            ompl_simple_setup_->getSpaceInformation();
//
//        bool tryMore = false;
//        if (ptc == false) tryMore = psf_->reduceVertices(pg);
//        if (ptc == false) psf_->collapseCloseVertices(pg);
//        int times = 0;
//        while (tryMore && ptc == false)
//        {
//          tryMore = psf_->reduceVertices(pg);
//          times++;
//        }
//        if (si->getStateSpace()->isMetricSpace())
//        {
//          if (ptc == false)
//            tryMore = psf_->shortcutPath(pg);
//          else
//            tryMore = false;
//          times = 0;
//          while (tryMore && ptc == false)
//          {
//            tryMore = psf_->shortcutPath(pg);
//            times++;
//          }
//        }
//
//        std::vector<ob::State*> &states = pg.getStates();
//        //	Calculate number of states required
//        unsigned int length = 0;
//        const int n1 = states.size() - 1;
//        for (int i = 0; i < n1; ++i)
//          length += si->getStateSpace()->validSegmentCount(states[i],
//              states[i + 1]);
//      }
//      pg.interpolate(50);
//      HIGHLIGHT_NAMED("OMPLSolver",
//          "Simplification took "<<ros::Duration(ros::Time::now()-start).toSec()<<"sec. States: "<<original_cnt<<"->"<<pg.getStateCount());
//    }
//    convertPath(pg, traj);
//
//    return SUCCESS;
//  }

  EReturn OMPLsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement(
        "TrajectorySmooth");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, smooth_);

    tmp_handle = handle.FirstChildElement("Solver");
    if (!ok(server_->registerParam<std_msgs::String>(ns_, tmp_handle, solver_)))
    {
      INDICATE_FAILURE
      return PAR_ERR;
    }

    tmp_handle = handle.FirstChildElement("SolverPackage");
    if (!ok(
        server_->registerParam<std_msgs::String>(ns_, tmp_handle,
            solver_package_)))
    {
      INDICATE_FAILURE
      return PAR_ERR;
    }

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
      ERROR(
          "EXOTica-OMPL plugin failed to load solver "<<solver_->data<<". \nError: %s" << ex.what());
    }
    if (!ok(base_solver_->initialiseBaseSolver(handle, server_)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
  }

  EReturn OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)

  {
    MotionSolver::specifyProblem(pointer);
    prob_ = boost::static_pointer_cast<OMPLProblem>(pointer);

    for (auto & it : prob_->getScenes())
    {
      if (!ok(it.second->activateTaskMaps()))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
    }
    if (!ok(base_solver_->specifyProblem(prob_)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
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

  EReturn OMPLsolver::setGoalState(const Eigen::VectorXd & qT, const double eps)
  {
    return base_solver_->setGoalState(qT, eps);
  }
} /* namespace exotica */
