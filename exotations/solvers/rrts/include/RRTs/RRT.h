/*
 * RRT.h
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_RRT_H_
#define EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_RRT_H_

#include "RRTs/SamplingProblem.h"
#include "RRTs/tree/tree.h"

namespace exotica
{
		//	Implementation of RRT
		class RRT: public MotionSolver
		{
			public:
				/*
				 * \brief	Default constructor
				 */
				RRT();

				/*
				 * \brief	Default destructor
				 */
				virtual ~RRT();

				/**
				 * \brief	Solves the problem
				 * @param	q0			Start state.
				 * @param	solution	This will be filled with the solution in joint space.
				 * @return	SUCESS if solution has been found, corresponding error code if not.
				 */
				EReturn Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution);

				/**
				 * \brief	Binds the solver to a specific problem which must be pre-initalised
				 * @param	pointer	Shared pointer to the motion planning problem
				 * @return	Successful if the problem is a valid AICOProblem
				 */
				virtual EReturn specifyProblem(PlanningProblem_ptr pointer);
			protected:
				/**
				 * \brief	Derived-elements initialiser: Pure Virtual
				 * @param	handle	XMLHandle to the Solver element
				 * @return	Should indicate success or otherwise
				 */
				virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
			private:
				RRTs::Tree_ptr tree_;	//	Tree
				SamplingProblem_ptr prob_; // Shared pointer to the planning problem.
		};
}
#endif /* EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_RRT_H_ */
