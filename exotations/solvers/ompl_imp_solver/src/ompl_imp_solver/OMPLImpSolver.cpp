/*
 *  Created on: 2 Mar 2016
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

#include "ompl_imp_solver/OMPLImpSolver.h"
#include <pluginlib/class_list_macros.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl_imp_solver/OMPLImplementationInitializer.h>

PLUGINLIB_EXPORT_CLASS(exotica::OMPLImpSolver, exotica::OMPLBaseSolver)

namespace exotica
{
  OMPLImpSolver::OMPLImpSolver() : OMPLBaseSolver("OMPLImpSolver"), algorithm_("geometric::RRTConnect"), margin_(0.0)
  {
    object_name_=algorithm_;
  }

  void OMPLImpSolver::initialiseSolver(Initializer& init)
  {
      OMPLImplementationInitializer prop(init);
      range_ = prop.Range;
      if(prop.Algorithm!="")
      {
        algorithm_ = "geometric::"+prop.Algorithm;
      }
      object_name_=algorithm_;

      if (known_algorithms_.find(algorithm_) != known_algorithms_.end())
      {
        HIGHLIGHT("Using planning algorithm "<<algorithm_);
      }
      else
      {
        ERROR("Unknown planning algorithm "<<algorithm_<<".");
        ERROR("Available algorithms: ");
        for (auto &it : known_algorithms_)
          ERROR(it.first);
      }
      margin_ = prop.Margin;
      timeout_ = prop.Timeout;
  }

  void OMPLImpSolver::specifyProblem(const SamplingProblem_ptr &prob)
  {
    prob_ = prob;
    base_type_ = prob_->getScene()->getBaseType();
    if (base_type_ == BASE_TYPE::FIXED)
      state_space_.reset(new OMPLRNStateSpace(prob_->getSpaceDim(), server_, prob_));
    else if (base_type_ == BASE_TYPE::FLOATING)
      state_space_.reset(new OMPLSE3RNStateSpace(prob_->getSpaceDim() - 6, server_, prob_));

    ompl_simple_setup_.reset(new og::SimpleSetup(state_space_));
    ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new OMPLStateValidityChecker(ompl_simple_setup_->getSpaceInformation(), prob_)));
    ompl_simple_setup_->setPlannerAllocator(boost::bind(known_algorithms_[algorithm_], _1, "Exotica_" + algorithm_));

    std::vector<int> project_vars = { 0, 1 };
    if (base_type_ == BASE_TYPE::FIXED)
      ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new OMPLRNProjection(state_space_, project_vars)));
    else if (base_type_ == BASE_TYPE::FLOATING)
      ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new OMPLSE3RNProjection(state_space_, project_vars)));
    ompl_simple_setup_->getSpaceInformation()->setup();
    ompl_simple_setup_->setup();
    if (ompl_simple_setup_->getPlanner()->params().hasParam("range"))
      ompl_simple_setup_->getPlanner()->params().setParam("range", range_);
  }

  bool OMPLImpSolver::solve(Eigen::VectorXdRefConst &x0, Eigen::MatrixXd &sol)
  {
    ros::Time startTime = ros::Time::now();
    finishedSolving_ = false;
    ompl::base::ScopedState<> ompl_start_state(state_space_);
    state_space_->as<OMPLBaseStateSpace>()->ExoticaToOMPLState(x0, ompl_start_state.get());
    ompl_simple_setup_->setStartState(ompl_start_state);
    preSolve();
    ompl::time::point start = ompl::time::now();
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout_ - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);

    if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      unregisterTerminationCondition();

      finishedSolving_ = true;

      if (!ompl_simple_setup_->haveSolutionPath()) return false;
      planning_time_ = ros::Time::now() - startTime;
      getSimplifiedPath(ompl_simple_setup_->getSolutionPath(), sol, ptc);
      planning_time_ = ros::Time::now() - startTime;
      postSolve();
      margin_ = init_margin_;
      prob_->update(Eigen::VectorXd(sol.row(sol.rows() - 1)), 0);
      prob_->getScene()->publishScene();
      return true;
    }
    else
    {
      finishedSolving_ = true;
      planning_time_ = ros::Time::now() - startTime;
      postSolve();
      return false;
    }
  }

  void OMPLImpSolver::convertPath(const og::PathGeometric &pg, Eigen::MatrixXd & traj)
  {
    traj.resize(pg.getStateCount(), prob_->getSpaceDim());
    Eigen::VectorXd tmp(prob_->getSpaceDim());

    for (int i = 0; i < (int) pg.getStateCount(); ++i)
    {
      state_space_->as<OMPLBaseStateSpace>()->OMPLToExoticaState(pg.getState(i), tmp);
      traj.row(i) = tmp;
    }
  }

  void OMPLImpSolver::getSimplifiedPath(og::PathGeometric &pg, Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc)
  {
    ros::Time start = ros::Time::now();

    og::PathSimplifierPtr psf_ = ompl_simple_setup_->getPathSimplifier();
    const ob::SpaceInformationPtr &si = ompl_simple_setup_->getSpaceInformation();

    bool tryMore = false;
    if (ptc == false) tryMore = psf_->reduceVertices(pg);
    if (ptc == false) psf_->collapseCloseVertices(pg);
    int times = 0;
    while (times < 10 && tryMore && ptc == false)
    {
      tryMore = psf_->reduceVertices(pg);
      times++;
    }
    if (si->getStateSpace()->isMetricSpace())
    {
      if (ptc == false)
        tryMore = psf_->shortcutPath(pg);
      else
        tryMore = false;
      while (times < 10 && tryMore && ptc == false)
      {
        tryMore = psf_->shortcutPath(pg);
        times++;
      }
    }

    std::vector<ob::State*> &states = pg.getStates();
    //  Calculate number of states required
    unsigned int length = 0;
    const int n1 = states.size() - 1;
    for (int i = 0; i < n1; ++i)
      length += si->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    //  unsigned int length = 50;
    pg.interpolate(length);
    convertPath(pg, traj);
    HIGHLIGHT("Trajectory simplification took "<<ros::Duration(ros::Time::now()-start).toSec()<<" sec. Trajectory length after interpolation = "<<length);
  }

  ompl::base::GoalPtr OMPLImpSolver::constructGoal()
  {
    return NULL;
  }

  std::string OMPLImpSolver::getPlannerName()
  {
    return planner_name_;
  }

  void OMPLImpSolver::setGoalState(Eigen::VectorXdRefConst qT, const double eps)
  {
    ompl::base::ScopedState<> gs(state_space_);
    state_space_->as<OMPLBaseStateSpace>()->ExoticaToOMPLState(qT, gs.get());
    init_margin_ = margin_;
    if (!ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
    {
      ERROR("Invalid goal state [Collision]\n"<<qT.transpose()<<", safety margin = "<<margin_);
      //  Try to reduce safety margin
      bool state_good = false;
      if (margin_ > 0)
      {
        unsigned int trial = 0;
        while (trial < 5)
        {
          margin_ /= 2.0;
          HIGHLIGHT("Retry with safety margin = "<<margin_);
          if (ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
          {
            state_good = true;
            break;
          }
          trial++;
        }
      }
      //  Last try
      if (!state_good)
      {
        margin_ = 0.0;
        HIGHLIGHT("Retry with safety margin = "<<margin_);
        if (ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
          state_good = true;
      }
      if (state_good)
      {
        HIGHLIGHT("Goal state passed collision check with safety margin = "<<margin_);
      }
      else
        throw_named("Goal state is not valid!");
    }

    if (!ompl_simple_setup_->getSpaceInformation()->satisfiesBounds(gs.get()))
    {
      state_space_->as<OMPLBaseStateSpace>()->stateDebug(qT);
      throw_named("Invalid goal state [Invalid joint bounds]\n"<<qT.transpose());
    }
    ompl_simple_setup_->setGoalState(gs, eps);
    margin_ = init_margin_;
  }

  void OMPLImpSolver::registerDefaultPlanners()
  {
    registerPlannerAllocator("geometric::RRT",
        boost::bind(&allocatePlanner<og::RRT>, _1, _2));
    registerPlannerAllocator("geometric::pRRT",
        boost::bind(&allocatePlanner<og::pRRT>, _1, _2));
    registerPlannerAllocator("geometric::RRTConnect",
        boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2));
    registerPlannerAllocator("geometric::LazyRRT",
        boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2));
//    registerPlannerAllocator("geometric::TRRT",
//        boost::bind(&allocatePlanner<og::TRRT>, _1, _2));
    registerPlannerAllocator("geometric::EST",
        boost::bind(&allocatePlanner<og::EST>, _1, _2));
    registerPlannerAllocator("geometric::SBL",
        boost::bind(&allocatePlanner<og::SBL>, _1, _2));
    registerPlannerAllocator("geometric::KPIECE",
        boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2));
    registerPlannerAllocator("geometric::BKPIECE",
        boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2));
    registerPlannerAllocator("geometric::LBKPIECE",
        boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2));
    registerPlannerAllocator("geometric::RRTStar",
        boost::bind(&allocatePlanner<og::RRTstar>, _1, _2));
    registerPlannerAllocator("geometric::PRM",
        boost::bind(&allocatePlanner<og::PRM>, _1, _2));
    registerPlannerAllocator("geometric::PRMstar",
        boost::bind(&allocatePlanner<og::PRMstar>, _1, _2));
    registerPlannerAllocator("geometric::PDST",
        boost::bind(&allocatePlanner<og::PDST>, _1, _2));
    registerPlannerAllocator("geometric::LazyPRM",
        boost::bind(&allocatePlanner<og::LazyPRM>, _1, _2));
    registerPlannerAllocator("geometric::SPARS",
        boost::bind(&allocatePlanner<og::SPARS>, _1, _2));
    registerPlannerAllocator("geometric::SPARStwo",
        boost::bind(&allocatePlanner<og::SPARStwo>, _1, _2));
    registerPlannerAllocator("geometric::LBTRRT",
        boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2));
    registerPlannerAllocator("geometric::STRIDE",
        boost::bind(&allocatePlanner<og::STRIDE>, _1, _2));
  }
}

