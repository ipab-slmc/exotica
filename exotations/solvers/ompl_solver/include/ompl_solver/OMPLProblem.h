/*
 * OMPLProblem.h
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#ifndef OMPLPROBLEM_H_
#define OMPLPROBLEM_H_

#include <exotica/PlanningProblem.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "task_definition/TaskTerminationCriterion.h"

namespace exotica
{

	class OMPLProblem : public PlanningProblem
	{
		public:
			OMPLProblem ();
			virtual
			~OMPLProblem ();

			int getSpaceDim();

			boost::mutex& getLock()
			{
				return update_lock_;
			}

			std::vector<TaskTerminationCriterion_ptr>& getGoals();
			std::vector<double>& getBounds();

            virtual EReturn reinitialise(rapidjson::Document& document, boost::shared_ptr<PlanningProblem> problem);

		protected:
			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle The handle to the XML-element describing the Problem Definition
			 * @return Indication of success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
		private:
			boost::mutex update_lock_;
			std::vector<TaskTerminationCriterion_ptr> goals_;
			std::vector<double> bounds_;
	};

	typedef boost::shared_ptr<exotica::OMPLProblem> OMPLProblem_ptr;

} /* namespace exotica */

#endif /* OMPLPROBLEM_H_ */
