/*
 * FRRTBase.h
 *
 *  Created on: 8 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBASE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBASE_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "exotica/EXOTica.hpp"
#include <ompl_solver/OMPLGoalSampler.h>
#include <ik_solver/ik_solver.h>
namespace ompl
{
	namespace geometric
	{
		class FRRTBase: public base::Planner
		{
			public:
				/*
				 * \brief	Default constructor
				 * @param	si		OMPL space information
				 */
				FRRTBase(const base::SpaceInformationPtr &si);

				/*
				 * \brief	Default destructor
				 */
				virtual ~FRRTBase();

				/*
				 * \brief	Get the planning data
				 * @param	data	Planning data
				 */
				virtual void getPlannerData(base::PlannerData &data) const;

				/*
				 * \brief	Solve the planning problem
				 * @param	ptc		Termination condition
				 * @return	OMPL planning status
				 */
				virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

				/*
				 * \brief	Clear planner information
				 */
				virtual void clear();

				/*
				 * \brief	Set up the planner
				 */
				virtual void setup();

				/*
				 * \brief	Set up the local planner (EXOTica)
				 * @param	xml_file	XML configuration file
				 * @param	scene		EXOTica scene
				 * @return	True if succeeded, false otherwise
				 */
				bool setUpLocalPlanner(const std::string & xml_file,
						const exotica::Scene_ptr & scene);

				bool resetSceneAndGoal(const exotica::Scene_ptr & scene,
						const Eigen::VectorXd & goal);
				/*
				 * \brief	Release memory
				 */
				void freeMemory();

				///	State sampler
				base::StateSamplerPtr sampler_;
				///	Random number generator
				RNG rng_;

			private:
				///	Local solver
				bool localSolve(const Eigen::VectorXd & qs, Eigen::VectorXd & qg, Eigen::MatrixXd & solution);
				exotica::Server_ptr ser_;
				exotica::IKsolver_ptr local_solver_;
				exotica::EParam<std_msgs::Float64> gTau_;
				exotica::EParam<std_msgs::Float64> lTau_;
		};

	}
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBASE_H_ */
