/*
 *  Created on: 22 Apr 2015
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

#include "frrt/FRRT.h"
namespace ompl
{
  namespace geometric
  {
    FRRT::FRRT(const base::SpaceInformationPtr &si)
        : FlexiblePlanner(si, "FRRT")
    {
      goalBias_ = 0.05;
      maxDist_ = 0.0;
      lastGoalMotion_ = NULL;
      Planner::declareParam<double>("range", this, &FRRT::setRange,
          &FRRT::getRange, "0.:1.:10000.");
      Planner::declareParam<double>("goal_bias", this, &FRRT::setGoalBias,
          &FRRT::getGoalBias, "0.:.05:1.");
      try_cnt_.resize(4);
      suc_cnt_.resize(4);
    }

    FRRT::~FRRT()
    {
      freeMemory();
    }

    void FRRT::clear()
    {
      Planner::clear();
      sampler_.reset();
      freeMemory();
      if (nn_) nn_->clear();
      lastGoalMotion_ = NULL;
      for (int i = 0; i < 4; i++)
        try_cnt_[i] = suc_cnt_[i] = 0;
    }

    void FRRT::setup()
    {
      Planner::setup();
      tools::SelfConfig sc(si_, getName());
      sc.configurePlannerRange(maxDist_);

      if (!nn_)
        nn_.reset(
            tools::SelfConfig::getDefaultNearestNeighbors<FM_RRT*>(
                si_->getStateSpace()));
      nn_->setDistanceFunction(
          boost::bind(&FRRT::distanceFunction, this, _1, _2));
    }

    void FRRT::freeMemory()
    {
      std::vector<FM_RRT*> motions;
      nn_->list(motions);
      for (unsigned int i = 0; i < motions.size(); ++i)
      {
        if (motions[i]->state) si_->freeState(motions[i]->state);
        delete motions[i];
      }
    }

    base::PlannerStatus FRRT::solve(
        const base::PlannerTerminationCondition &ptc)
    {
      if (!local_solver_)
      {
        INDICATE_FAILURE
        return base::PlannerStatus::CRASH;
      }
      nn_->clear();
      checkValidity();
      base::Goal *goal = pdef_->getGoal().get();
      base::GoalSampleableRegion *goal_s =
          dynamic_cast<base::GoalSampleableRegion*>(goal);
      FM_RRT *init_motion = NULL;
      while (const base::State *st = pis_.nextStart())
      {
        FM_RRT *motion = new FM_RRT(si_);
        si_->copyState(motion->state, st);
        if (init_motion == NULL)
        {
          init_motion = new FM_RRT(si_);
          si_->copyState(init_motion->state, st);
        }
        nn_->add(motion);
      }
      FM_RRT *goal_motion = NULL;
      double nearest_r = 0;
      if (goal_s && goal_s->canSample())
      {
        FM_RRT *tmpgoal = new FM_RRT(si_);
        goal_s->sampleGoal(tmpgoal->state);

        if (!goal->isSatisfied(tmpgoal->state))
        {
          ERROR("FRRT, Invalid goal state");
          return base::PlannerStatus::INVALID_GOAL;
        }
        goal_motion = tmpgoal;
        nearest_r = 0.05 * distanceFunction(init_motion, goal_motion);
      }

      if (nn_->size() == 0)
      {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        INDICATE_FAILURE
        return base::PlannerStatus::INVALID_START;
      }
      if (!sampler_) sampler_ = si_->allocStateSampler();
      OMPL_INFORM(
          "%s: Starting planning with %u states already in datastructure",
          getName().c_str(), nn_->size());

      ///	Solution info
      FM_RRT *solution = NULL;
      bool solved = false;
      for (int i = 0; i < 4; i++)
        try_cnt_[i] = suc_cnt_[i] = 0;

      ///Lets do it clean and nice !!!!!!!
      bool newTry = true;
      FM_RRT *start_motion = init_motion;
      static bool first_check = true;
      while (ptc == false)
      {
        /// Move from start to goal
        FM_RRT *new_motion = new FM_RRT(si_);
        if (newTry)
        {
          newTry = false;
          ompl::base::State *last_valid_state = si_->allocState();
          std::pair<ompl::base::State*, double> last_valid(last_valid_state, 0);
          bool valid = false;
          if (!first_check) try_cnt_[3]++;
          if (!first_check
              && si_->checkMotion(start_motion->state, goal_motion->state,
                  last_valid))
          {
            new_motion = goal_motion;
            valid = true;
            suc_cnt_[3]++;
          }
          if (first_check) first_check = false;
          if (!valid)
          {
            try_cnt_[0]++;
            if (last_valid.second == 0) last_valid_state = NULL;
            local_map_->jointRef = global_goal_;
            local_solver_->getProblem()->setTau(gTau_->data);
            if (localSolve(start_motion, last_valid_state, new_motion))
            {
              valid = true;
              suc_cnt_[0]++;
            }
            else if (new_motion->internal_path)
            {
              new_motion->parent = start_motion;
              nn_->add(new_motion);
            }
          }

          if (valid)
          {
            new_motion->parent = start_motion;
            nn_->add(new_motion);

            double dist = 0.0;
            bool sat = goal->isSatisfied(new_motion->state, &dist);
            if (sat)
            {
              solution = new_motion;
              break;
            }
            else
            {
              WARNING_NAMED("FRRT", "Goal check failed with error "<<dist);
            }
          }
        }
        ///	If global not succeeded
        else
        {
          ///	Sample a random state
          bool r_ok = false;
          do
          {
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            {
              goal_s->sampleGoal(new_motion->state);
            }
            else
            {
              sampler_->sampleUniform(new_motion->state);
            }
            r_ok = si_->getStateValidityChecker()->isValid(new_motion->state);
          } while (!r_ok && ptc == false);

          if (!r_ok)
          {
            OMPL_ERROR("Random sample failed");
            break;
          }
          FM_RRT *near_motion = nn_->nearest(new_motion);

          bool valid = false;

          ///	Do a regular line segment check
          ompl::base::State *last_valid_state = si_->allocState();
//					si_->copyState(last_valid_state, near_motion->state);
          std::pair<ompl::base::State*, double> last_valid(last_valid_state, 0);
          try_cnt_[2]++;
          if (si_->checkMotion(near_motion->state, new_motion->state,
              last_valid))
          {
            suc_cnt_[2]++;
            new_motion->parent = near_motion;
            nn_->add(new_motion);
            valid = true;
          }
          ///	Do a local try
          else
          {

            if (last_valid.second == 0) last_valid_state = NULL;
            try_cnt_[1]++;
            ///	Set local solver goal
            Eigen::VectorXd eigen_g((int) si_->getStateDimension());
            copyStateToEigen(new_motion->state, eigen_g);
            local_map_->jointRef = eigen_g;
            local_solver_->getProblem()->setTau(lTau_->data);
            if (localSolve(near_motion, last_valid_state, new_motion))
            {
              suc_cnt_[1]++;
              new_motion->parent = near_motion;
              nn_->add(new_motion);
              valid = true;
            }
            else if (new_motion->internal_path)
            {

              new_motion->parent = start_motion;
              nn_->add(new_motion);
            }
          }

          if (valid)
          {
            newTry = true;
            start_motion = new_motion;
          }
        }

      }

      if (solution != NULL)
      {
        lastGoalMotion_ = solution;
        /* construct the solution path */
        std::vector<FM_RRT*> mpath;
        while (solution != NULL)
        {
          if (solution->internal_path != nullptr)
          {
            for (int i = solution->internal_path->rows() - 1; i > 0; i--)
            {
              FM_RRT *local_motion = new FM_RRT(si_);
              Eigen::VectorXd tmp = solution->internal_path->row(i);
              copyEigenToState(tmp, local_motion->state);
              mpath.push_back(local_motion);
            }
            if (solution->inter_state != NULL)
            {
              FM_RRT *local_motion = new FM_RRT(si_);
              si_->copyState(local_motion->state, solution->inter_state);
              mpath.push_back(local_motion);
            }
          }
          else
          {
            mpath.push_back(solution);
          }
          solution = solution->parent;
        }
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1; i >= 0; --i)
          path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), false, 0, getName());
        OMPL_INFORM("Problem Solved");
        solved = true;
      }
      WARNING_NAMED("FRRT", "Created "<<nn_->size()<<" states");
      WARNING_NAMED("FRRT",
          "Global succeeded/try "<<suc_cnt_[0]<<"/"<<try_cnt_[0]);
      WARNING_NAMED("FRRT",
          "GNormal succeeded/try "<<suc_cnt_[3]<<"/"<<try_cnt_[3]);
      WARNING_NAMED("FRRT",
          "Local succeeded/try "<<suc_cnt_[1]<<"/"<<try_cnt_[1]);
      WARNING_NAMED("FRRT",
          "Normal succeeded/try "<<suc_cnt_[2]<<"/"<<try_cnt_[2]);
      first_check = true;
      return base::PlannerStatus(solved, false);
    }

    void FRRT::getPlannerData(base::PlannerData &data) const
    {
      Planner::getPlannerData(data);
      std::vector<FM_RRT*> motions;
      if (nn_) nn_->list(motions);
      if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
      for (unsigned int i = 0; i < motions.size(); ++i)
      {
        if (motions[i]->parent == NULL)
          data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
          data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
              base::PlannerDataVertex(motions[i]->state));
      }
      data.properties["GlobalSolveTry"] = std::to_string(try_cnt_[0]);
      data.properties["GlobalSolveSuccess"] = std::to_string(suc_cnt_[0]);
      data.properties["GlobalCheckTry"] = std::to_string(try_cnt_[3]);
      data.properties["GlobalCheckSuccess"] = std::to_string(suc_cnt_[3]);
      data.properties["LocalSolveTry"] = std::to_string(try_cnt_[1]);
      data.properties["LocalSolveSuccess"] = std::to_string(suc_cnt_[1]);
      data.properties["LocalCheckTry"] = std::to_string(try_cnt_[2]);
      data.properties["LocalCheckSuccess"] = std::to_string(suc_cnt_[2]);
    }

    bool FRRT::localSolve(FM_RRT *sm, base::State *is, FM_RRT *gm)
    {
      int dim = (int) si_->getStateDimension();
      Eigen::VectorXd qs(dim), qg(dim);
      copyStateToEigen(is == NULL ? sm->state : is, qs);
      Eigen::MatrixXd local_path;
      if (FlexiblePlanner::localSolve(qs, qg, local_path))
      {
        /* Local planner succeeded */
        gm->inter_state = is;
        gm->internal_path.reset(new Eigen::MatrixXd(local_path));
        gm->parent = sm;
        qg = local_path.row(local_path.rows() - 1).transpose();
        copyEigenToState(qg, gm->state);
        return true;
      }
      return false;
    }
  }	//	Namespace geometric
}	//	Namespace ompl

