/*
 * OMPLProjection.h
 *
 *  Created on: 20 May 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_

#include <ompl/base/ProjectionEvaluator.h>
#include "ompl_solver/OMPLStateSpace.h"
///	Implementation of OMPL EXOTica Projection (Testing)
namespace exotica
{
	class OMPLProjection: public ompl::base::ProjectionEvaluator
	{
		public:
			OMPLProjection(const ompl::base::StateSpacePtr &space, const std::vector<int> & vars) :
					ompl::base::ProjectionEvaluator(space), variables_(vars)
			{

			}

			~OMPLProjection()
			{
				//TODO
			}

			virtual unsigned int getDimension(void) const
			{
				return variables_.size();
			}

			virtual void defaultCellSizes()
			{
				cellSizes_.clear();
				cellSizes_.resize(variables_.size(), 0.1);
			}

			virtual void project(const ompl::base::State *state,
					ompl::base::EuclideanProjection &projection) const
			{
				for (std::size_t i = 0; i < variables_.size(); ++i)
					projection(i) =
							state->as<exotica::OMPLStateSpace::StateType>()->values[variables_[i]];
			}

		private:
			std::vector<int> variables_;
	};
}
#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_ */
