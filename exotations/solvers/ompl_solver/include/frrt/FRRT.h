/*
 * FRRT.h
 *
 *  Created on: 22 Apr 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "exotica/EXOTica.hpp"
#include <ompl_solver/OMPLGoal.h>
#include <ik_solver/ik_solver.h>
namespace ompl
{
	namespace geometric
	{
		/*
		 * \brief	Implementation of Flexible RRT Planning Algorithm
		 */
		class FRRT: public base::Planner
		{
			public:
				enum LocalMode
				{
					GLOBAL = 0, LOCAL = 10, GOAL_SAMPLER = 20
				};
				/*
				 * \brief	Default constructor
				 * @param	si		OMPL space information
				 */
				FRRT(const base::SpaceInformationPtr &si);

				/*
				 * \brief	Default destructor
				 */
				virtual ~FRRT();

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
				 * \brief	Set goal bias value
				 * @param	bias	Goal bias
				 */
				void setGoalBias(double bias)
				{
					goalBias_ = bias;
				}

				/*
				 * \brief	Get goal bias value
				 * @return	Goal bias
				 */
				double getGoalBias() const
				{
					return goalBias_;
				}

				/*
				 * \brief	Set maximum distance
				 * @param	dist	Maximum distance
				 */
				void setRange(double dist)
				{
					maxDist_ = dist;
				}

				/*
				 * \brief	Get maximum distance
				 * @return	Maximum distance
				 */
				double getRange() const
				{
					return maxDist_;
				}

				template<template<typename T> class NN>
				void setNearestNeighbors()
				{
					nn_.reset(new NN<Motion*>());
				}

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

				bool resetScene(const exotica::Scene_ptr & scene);
			protected:
				/*
				 * \brief	Motion class. This is where all the planner specific parameters are defined
				 */
				class Motion
				{
					public:
						/*
						 * \brief	Constructor
						 */
						Motion() :
								state(NULL), parent(NULL), global_valid_(true)
						{
						}

						Motion(const base::SpaceInformationPtr &si) :
								state(si->allocState()), parent(NULL), global_valid_(true)
						{

						}
						/*
						 * \brief	Destructor
						 */
						~Motion()
						{

						}

						void setGlobalInvalid()
						{
							global_valid_ = false;
						}

						bool isGlobalValid()
						{
							return global_valid_;
						}
						///	The OMPL state
						base::State *state;
						///	The parent node
						Motion *parent;
						///	The internal flexible path
						boost::shared_ptr<Eigen::MatrixXd> internal_path;
					private:
						bool global_valid_;
				};

				/*
				 * \brief	Release memory
				 */
				void freeMemory();

				/*
				 * \brief	Compute distance between motions (actually distance between contained states)
				 * @param	a		Motion a
				 * @param	b		Motion b
				 * @return	Distance between a and b
				 */
				double distanceFunction(const Motion *a, const Motion *b) const
				{
					return si_->distance(a->state, b->state);
				}

				///	State sampler
				base::StateSamplerPtr sampler_;
				///	The tree
				boost::shared_ptr<NearestNeighbors<Motion*> > nn_;
				/// Goal bias
				double goalBias_;
				///	Maximum distance
				double maxDist_;
				///	Random number generator
				RNG rng_;
				///	Last goal
				Motion *lastGoalMotion_;

			private:
				///	Local solver
				bool localSolve(Motion *sm, Motion *gm, LocalMode mode);

				exotica::IKsolver_ptr local_solver_;

				///	Store the local taskmap and task
				exotica::TaskSqrError_ptr local_task_;
				exotica::TaskSqrError_ptr global_task_;
				exotica::TaskSqrError_ptr collision_task_;

				///	Store initial rhos
				std::vector<double> init_rho_;
				///	For analyse
				std::vector<int> try_cnt_;
				std::vector<int> suc_cnt_;

				int global_try_;
				int global_succeeded_;
				int local_try_;
				int local_succeeded_;
				int normal_try_;
				int normal_succeeded_;
		};
	}
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_ */
