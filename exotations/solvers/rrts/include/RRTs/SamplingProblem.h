/*
 * SamplingProblem.h
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_SAMPLINGPROBLEM_H_
#define EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_SAMPLINGPROBLEM_H_

#include <exotica/PlanningProblem.h>

namespace exotica
{
	class SamplingProblem: public PlanningProblem
	{
		public:
			SamplingProblem();
			virtual ~SamplingProblem();

		protected:
			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle[in] The handle to the XML-element describing the Problem Definition
			 * @return           Indication of success/failure: TODO
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;
	};
	typedef boost::shared_ptr<exotica::SamplingProblem> SamplingProblem_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_SAMPLINGPROBLEM_H_ */
