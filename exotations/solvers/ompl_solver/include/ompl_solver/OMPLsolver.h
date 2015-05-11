/*
 * OMPLsolver.h
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#ifndef OMPLSOLVER_H_
#define OMPLSOLVER_H_

#include <exotica/MotionSolver.h>
#include "ompl_solver/OMPLProblem.h"
#include "ompl_solver/OMPLStateSpace.h"
#include "ompl_solver/common.h"
#include "ompl_solver/OMPLStateValidityChecker.h"
#include "ompl_solver/OMPLGoalSampler.h"
#include <ompl/geometric/PathSimplifier.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

namespace exotica
{

	class OMPLsolver : public MotionSolver, public boost::enable_shared_from_this<OMPLsolver>
	{
		public:
			OMPLsolver ();
			virtual
			~OMPLsolver ();

			/**
			 * \brief Binds the solver to a specific problem which must be pre-initalised
			 * @param pointer Shared pointer to the motion planning problem
			 * @return        Successful if the problem is a valid AICOProblem
			 */
			virtual EReturn specifyProblem(PlanningProblem_ptr pointer);

			std::vector<std::string> getPlannerNames();

			/**
			 * \brief Solves the problem
			 * @param q0 Start state.
			 * @param solution This will be filled with the solution in joint space.
			 * @return SUCESS if solution has been found, corresponding error code if not.
			 */
			EReturn Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution);

			/**
			 * \brief Terminates planning
			 */
			bool terminate();

			const boost::shared_ptr<OMPLStateSpace> getOMPLStateSpace() const
			{
				return state_space_;
			}

			const boost::shared_ptr<og::SimpleSetup> getOMPLSimpleSetup() const
			{
				return ompl_simple_setup_;
			}

			const std::string getAlgorithm()
			{
				return selected_planner_;
			}

			const OMPLProblem_ptr getProblem() const
			{
				return prob_;
			}

			bool getFinishedSolving();

			int getGoalMaxAttempts();

			void setMaxPlanningTime(double t);

            ros::Duration planning_time_;

		protected:
			/**
			 * \brief Registers default planners
			 */
			void registerDefaultPlanners();

		private:
			/**
			 * \brief Derived-elements initialiser: Pure Virtual
			 * @param handle XMLHandle to the Solver element
			 * @return       Should indicate success or otherwise
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

			/**
			 * \biref Registers a planner allocator
			 * @param planner_id Planner name
			 * @param pa Planner allocation function
			 */
			void registerPlannerAllocator(const std::string &planner_id, const ConfiguredPlannerAllocator &pa)
			{
				known_planners_[planner_id] = pa;
			}

			/**
			 * \biref Converts OMPL trajectory into Eigen Matrix
			 * @param pg OMPL trajectory
			 * @param traj Eigen Matrix trajectory
			 * @return Indicates success
			 */
			EReturn convertPath(const ompl::geometric::PathGeometric &pg, Eigen::MatrixXd & traj);

			EReturn getSimplifiedPath(ompl::geometric::PathGeometric &pg, Eigen::MatrixXd & traj);
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
			 * \brief Starts goal sampling
			 */
			void startSampling();
			/**
			 * \brief Stops goal sampling
			 */
			void stopSampling();
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
			ompl::base::GoalPtr constructGoal();

			OMPLProblem_ptr prob_; //!< Shared pointer to the planning problem.

			/// \brief List of all known OMPL planners
			std::map<std::string, ConfiguredPlannerAllocator> known_planners_;

			/// The OMPL planning context; this contains the problem definition and the planner used
			boost::shared_ptr<og::SimpleSetup> ompl_simple_setup_;

			/// \brief OMPL state space specification
			boost::shared_ptr<OMPLStateSpace> state_space_;

			/// \brief Planner name
			std::string selected_planner_;

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


	};

	typedef boost::shared_ptr<exotica::OMPLsolver> OMPLsolver_ptr;

} /* namespace exotica */

#endif /* OMPLSOLVER_H_ */
