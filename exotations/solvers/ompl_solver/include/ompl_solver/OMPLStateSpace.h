/*
 * OMPLStateSpace.h
 *
 *  Created on: 9 Jul 2014
 *      Author: s0972326
 */

#ifndef OMPLSTATESPACE_H_
#define OMPLSTATESPACE_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl_solver/OMPLProblem.h"

namespace exotica
{

	class OMPLStateSpace : public ompl::base::RealVectorStateSpace
	{
		public:

		  static boost::shared_ptr<OMPLStateSpace> FromProblem(OMPLProblem_ptr prob);

			OMPLStateSpace (unsigned int dim = 0);
			virtual
			~OMPLStateSpace ();

			EReturn copyToOMPLState(ompl::base::State *state, Eigen::VectorXd q) const;
			EReturn copyFromOMPLState(const ompl::base::State *state, Eigen::VectorXd& q) const;
	};

} /* namespace exotica */

#endif /* OMPLSTATESPACE_H_ */
