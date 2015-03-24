/***********************************************************************\
|    The base class for all Termination Criteria. Since these can be    |
 |  quite variable the base implementation only defines the initialisers |
 |  and the form of the query function.                                  |
 |                                                                       |
 |           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
 |                    Last Edited: 19 - March - 2014                     |
 \***********************************************************************/
#ifndef EXOTICA_TASK_TERMINATION_CRITERION_H
#define EXOTICA_TASK_TERMINATION_CRITERION_H

#include "exotica/TaskDefinition.h"//!< The Component base
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Tools.h"        //!< For XML-Parsing/ErrorFunction definition

/**
 * \brief Convenience registrar for the Termination Criterion Type
 */
#define REGISTER_ENDCRITERION_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::Terminator, TYPE, DERIV)

namespace exotica
{
	class TaskTerminationCriterion: public TaskDefinition
	{
		public:
			/**
			 * \brief Default Constructor
			 */
			TaskTerminationCriterion();
			virtual ~TaskTerminationCriterion();

			/**
			 * \brief	Evaluate the termination criterion
			 * @param	x	Robot configuration
			 * @param	t	Time index
			 */
			virtual EReturn evaluate(const Eigen::VectorXd & x, const int t) = 0;

			/**
			 * \brief Terminate Query: PURE VIRTUAL
			 * @return	Termination status
			 */
			virtual ETerminate terminate() = 0;

		protected:
			/**
			 * \brief Derived-Initialisation
			 * @param handle XML handle for any derived parameters
			 * @return       Should indicate success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;

			/*
			 * \brief	Set goal
			 */
			EReturn setGoal(const Eigen::Ref<Eigen::VectorXd> & goal);
		private:
			ETerminate status_;        //!< The 'status' of the termination criterion
			boost::mutex lock_;   //!< Lock for thread-safety
			Eigen::VectorXd goal_;	//	Goal
	};

	typedef exotica::Factory<std::string, exotica::TaskTerminationCriterion> TerminationCriterionCreator; //!< Convenience name for the EndCriterion Singleton Factory
}
#endif
