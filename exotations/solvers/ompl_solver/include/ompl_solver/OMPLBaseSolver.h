/*
 * OMPLBaseSolver.h
 *
 *  Created on: 22 Feb 2016
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_

#include "ompl_solver/OMPLProblem.h"
#include "ompl_solver/common.h"
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

namespace exotica
{
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  class OMPLBaseSolver
  {
    public:
      virtual ~OMPLBaseSolver();
      virtual EReturn initialise(tinyxml2::XMLHandle & handle,
          const OMPLProblem_ptr &prob) = 0;
      virtual EReturn solve(const Eigen::VectorXd &x0,
          Eigen::MatrixXd &sol) = 0;
      virtual EReturn setGoalState(const Eigen::VectorXd & qT,
          const double eps = std::numeric_limits<double>::epsilon()) = 0;

      std::string getAlgorithm();
      double getPlanningTime();
    protected:
      OMPLBaseSolver();

    private:
      virtual void setAlgorithmName() = 0;

      /**
       * \biref Converts OMPL trajectory into Eigen Matrix
       * @param pg OMPL trajectory
       * @param traj Eigen Matrix trajectory
       * @return Indicates success
       */
      virtual EReturn convertPath(const og::PathGeometric &pg,
          Eigen::MatrixXd & traj) = 0;

      virtual EReturn getSimplifiedPath(og::PathGeometric &pg,
          Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc) = 0;
      /**
       * \brief Registers trajectory termination condition
       * @param ptc Termination criteria
       */
      void registerTerminationCondition(
          const ob::PlannerTerminationCondition &ptc);

      /**
       * \brief Unregisters trajectory termination condition
       */
      void unregisterTerminationCondition();

      /**
       * \brief Executes pre solve steps
       */
      void preSolve();
      /**
       * \brief Executes post solve steps
       */
      void postSolve();

      /**
       * Constructs goal representation
       */
      virtual ompl::base::GoalPtr constructGoal() = 0;

      /**
       * \brief Record data for analysis
       */
      void recordData();

      OMPLProblem_ptr prob_; //!< Shared pointer to the planning problem.

      /// The OMPL planning context; this contains the problem definition and the planner used
      boost::shared_ptr<og::SimpleSetup> ompl_simple_setup_;

      /// \brief OMPL state space specification
      boost::shared_ptr<ob::StateSpace> state_space_;

      /// \brief Maximum allowed time for planning
      double timeout_;

      /// \brief Planner termination criteria
      const ob::PlannerTerminationCondition *ptc_;
      /// \brief Planner termination criteria mutex
      boost::mutex ptc_lock_;

      /// \bierf True when the solver finished its work and background threads need to finish.
      bool finishedSolving_;

      // \brief Max number of attempts at sampling the goal region
      int goal_ampling_max_attempts_;

      /// \brief  Porjection type
      std::string projector_;

      /// \brief  Projection components
      std::vector<std::string> projection_components_;

      ros::Duration planning_time_;

      std::string algorithm_;
  };
  typedef boost::shared_ptr<exotica::OMPLBaseSolver> OMPLBaseSolver_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLBASESOLVER_H_ */
