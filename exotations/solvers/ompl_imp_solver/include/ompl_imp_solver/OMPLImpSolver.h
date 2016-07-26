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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLIMPSOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLIMPSOLVER_H_

#include "ompl_solver/OMPLBaseSolver.h"
#include "ompl_imp_solver/OMPLRNStateSpace.h"
#include "ompl_imp_solver/OMPLSE3RNStateSpace.h"
#include <pluginlib/class_loader.h>

namespace exotica
{
  /*
   * The implementation of basic EXOTica-OMPL solver
   */
  class OMPLImpSolver: public OMPLBaseSolver
  {
    public:
      /*
       * \brief Default constructor
       */
      OMPLImpSolver();

      /*
       * \brief Initialisation function
       * @param handle  XML handle that contains solver specific parameters
       */
      virtual void initialiseSolver(tinyxml2::XMLHandle & handle);

      /*
       * \brief Solve function
       * @param x0      start configuration
       * @param sol     Solution
       */
      virtual bool solve(const Eigen::VectorXd &x0, Eigen::MatrixXd &sol);

      /*
       * \brief Set goal state to the sampled state space
       * @param qT      Goal state in sampled state space
       * @param eps     Numerical tolerance
       */
      virtual void setGoalState(const Eigen::VectorXd & qT, const double eps =
          std::numeric_limits<double>::epsilon());

      /*
       * \brief Get the planner's name
       * @return  Planner's name
       */
      virtual std::string & getPlannerName();

      /*
       * \brief Assign exotica problem to this solver
       * @param prob    EXOTica Planning Problem pointer
       */
      virtual void specifyProblem(const OMPLProblem_ptr &prob);
    protected:
      /*
       * \brief Register default ompl planning algorithms
       */
      virtual void registerDefaultPlanners();

      /*
       * \brief Allocate state space sampler
       */
      ompl::base::StateSamplerPtr allocStateSampler(
          const ompl::base::StateSpace *ss);

      /*
       * \biref Converts OMPL trajectory into Eigen Matrix
       * @param pg      OMPL trajectory
       * @param traj    Eigen Matrix trajectory
       */
      virtual void convertPath(const og::PathGeometric &pg,
          Eigen::MatrixXd & traj);

      /*
       * \brief Simplifying the trajectory
       * @param pg      OMPL geometric path
       * @param
       */
      virtual void getSimplifiedPath(og::PathGeometric &pg,
          Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc);
      virtual ompl::base::GoalPtr constructGoal();
      std::string algorithm_;
      std::string range_;
      std::string object_name_;
      EParam<std_msgs::Float64> margin_;
      Eigen::VectorXd qT_;
    private:
      BASE_TYPE base_type_;
      double init_margin_;
  };
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLIMPSOLVER_H_ */
