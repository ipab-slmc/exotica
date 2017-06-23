/*
 *  Created on: 22 Feb 2016
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_

#include "exotica/Problems/SamplingProblem.h"
#include "ompl_solver/common.h"
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <exotica/Property.h>

namespace exotica
{
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  class OMPLBaseSolver
  {
    public:
      virtual ~OMPLBaseSolver();

      void initialiseBaseSolver(Initializer& init);
      virtual void initialiseSolver(Initializer& init) = 0;

      virtual bool solve(Eigen::VectorXdRefConst x0, Eigen::MatrixXd &sol) = 0;
      virtual void setGoalState(Eigen::VectorXdRefConst qT, const double eps = std::numeric_limits<double>::epsilon()) = 0;

      virtual std::string getPlannerName() = 0;
      double getPlanningTime();

      const og::SimpleSetupPtr getOMPLSimpleSetup() const;

      virtual void specifyProblem(const SamplingProblem_ptr &prob) = 0;

      static pluginlib::ClassLoader<exotica::OMPLBaseSolver> base_solver_loader;
    protected:
      OMPLBaseSolver(const std::string planner_name);
      void registerPlannerAllocator(const std::string &planner_id, const ConfiguredPlannerAllocator &pa)
      {
        known_algorithms_[planner_id] = pa;
      }

      template<typename T> static ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si, const std::string &new_name)
      {
        ompl::base::PlannerPtr planner(new T(si));
        if (!new_name.empty()) planner->setName(new_name);
        return planner;
      }

      std::map<std::string, ConfiguredPlannerAllocator> known_algorithms_;
      std::string planner_name_;

      virtual void registerDefaultPlanners() = 0;
      /**
       * \biref Converts OMPL trajectory into Eigen Matrix
       * @param pg OMPL trajectory
       * @param traj Eigen Matrix trajectory
       * @return Indicates success
       */
      virtual void convertPath(const og::PathGeometric &pg, Eigen::MatrixXd & traj) = 0;

      virtual void getSimplifiedPath(og::PathGeometric &pg, Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc) = 0;
      /**
       * \brief Registers trajectory termination condition
       * @param ptc Termination criteria
       */
      void registerTerminationCondition(const ob::PlannerTerminationCondition &ptc);

      /**
       * \brief Unregisters trajectory termination condition
       */
      void unregisterTerminationCondition();

      /**
       * \brief Executes pre solve steps
       */
      bool virtual preSolve();
      /**
       * \brief Executes post solve steps
       */
      bool virtual postSolve();

      /**
       * Constructs goal representation
       */
      virtual ompl::base::GoalPtr constructGoal() = 0;

      /**
       * \brief Record data for analysis
       */
      void recordData();

      SamplingProblem_ptr prob_; //!< Shared pointer to the planning problem.

      /// The OMPL planning context; this contains the problem definition and the planner used
      og::SimpleSetupPtr ompl_simple_setup_;

      /// \brief OMPL state space specification
      ompl::base::StateSpacePtr state_space_;

      /// \brief Maximum allowed time for planning
      double timeout_;

      /// \brief Planner termination criteria
      const ob::PlannerTerminationCondition *ptc_;

      /// \bierf True when the solver finished its work and background threads need to finish.
      bool finishedSolving_;

      // \brief Max number of attempts at sampling the goal region
      int goal_sampling_max_attempts_;

      /// \brief  Porjection type
      std::string projector_;

      /// \brief  Projection components
      std::vector<std::string> projection_components_;

      ros::Duration planning_time_;

  };
  typedef boost::shared_ptr<exotica::OMPLBaseSolver> OMPLBaseSolver_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_ */
