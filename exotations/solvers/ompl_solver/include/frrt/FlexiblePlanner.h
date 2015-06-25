/*
 * FlexiblePlanner.h
 *
 *  Created on: 8 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "exotica/EXOTica.hpp"
#include <ompl_solver/OMPLGoalSampler.h>
#include <ik_solver/ik_solver.h>
namespace ompl
{
	namespace geometric
	{
		/*
		 * \brief	Implementation of FlexiblePlanner base, mainly designed to manage the local solver in a systematic way.
		 * 			So that the actual algorithms can call 'localSolve' directly
		 */
		class FlexiblePlanner: public base::Planner
		{
			public:
				/*
				 * \brief	Default constructor
				 * @param	si		OMPL space information
				 * @param	name	Planner name
				 */
				FlexiblePlanner(const base::SpaceInformationPtr &si, const std::string & name);

				/*
				 * \brief	Default destructor
				 */
				virtual ~FlexiblePlanner();

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

				///	State sampler
				base::StateSamplerPtr sampler_;
				///	Random number generator
				RNG rng_;

				int checkCnt_;
			protected:
				///	Local solver
				exotica::EReturn localSolve(const Eigen::VectorXd & qs, Eigen::VectorXd & qg, Eigen::MatrixXd & solution);
				exotica::Server_ptr ser_;
				exotica::IKsolver_ptr local_solver_;
				boost::shared_ptr<exotica::Identity> local_map_;
				exotica::EParam<std_msgs::Float64> gTau_;
				exotica::EParam<std_msgs::Float64> lTau_;
				Eigen::VectorXd global_goal_;
		};

	}
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_ */
