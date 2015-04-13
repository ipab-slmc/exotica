/*
 * OMPLGoalUnion.h
 *
 *  Created on: 10 Jul 2014
 *      Author: s0972326
 */

#ifndef OMPLGOALUNION_H_
#define OMPLGOALUNION_H_

#include <ompl/base/goals/GoalSampleableRegion.h>

namespace exotica
{

	class OMPLGoalUnion : public ompl::base::GoalSampleableRegion
	{
		public:
			/** @brief Constructor
						   *  @param goals The input set of goals*/
			OMPLGoalUnion (const std::vector<ompl::base::GoalPtr> &goals);
			virtual
			~OMPLGoalUnion ();

			/** @brief Sample a goal*/
			virtual void sampleGoal(ompl::base::State *st) const;

			/** @brief Get the max sample count*/
			virtual unsigned int maxSampleCount() const;

			/** @brief Query if sampler can find any sample*/
			virtual bool canSample() const;

			/** @brief Query if sampler could find a sample in the future */
			virtual bool couldSample() const;

			/** @brief Is the goal satisfied for this state (given a distance)*/
			virtual bool isSatisfied(const ompl::base::State *st, double *distance) const;

			/** @brief Find the distance of this state from the goal*/
			virtual double distanceGoal(const ompl::base::State *st) const;

			/** @brief If there are any member lazy samplers, start them */
			void startSampling();

			/** @brief If there are any member lazy samplers, stop them */
			void stopSampling();

			/** @brief Pretty print goal information*/
			virtual void print(std::ostream &out = std::cout) const;

		protected:

			std::vector<ompl::base::GoalPtr> goals_;
			mutable unsigned int             gindex_;
	};

} /* namespace exotica */

#endif /* OMPLGOALUNION_H_ */
