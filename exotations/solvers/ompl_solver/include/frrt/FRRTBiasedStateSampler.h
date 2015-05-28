/*
 * FRRTStateSampler.h
 *
 *  Created on: 25 May 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBIASEDSTATESAMPLER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBIASEDSTATESAMPLER_H_

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateSampler.h>

namespace ompl
{
	namespace base
	{
		class FRRTBiasedStateSampler : public ValidStateSampler
		{
			public:
				FRRTBiasedStateSampler(const SpaceInformation *si);
				virtual ~FRRTBiasedStateSampler();
				
				
		};
	}
}


#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTBIASEDSTATESAMPLER_H_ */
