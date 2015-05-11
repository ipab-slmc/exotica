/*
 * ik_problem.h
 *
 *  Created on: 15 Jul 2014
 *      Author: yiming
 */

#ifndef IK_PROBLEM_H_
#define IK_PROBLEM_H_
#include <exotica/PlanningProblem.h>
#include "task_definition/TaskSqrError.h"

namespace exotica
{
	/**
	 * IK problem implementation
	 */
	class IKProblem: public PlanningProblem
	{
		public:
			IKProblem();
			virtual ~IKProblem();

			/**
			 * \brief	Get configuration weight
			 * @return	configuration weight
			 */
			Eigen::MatrixXd getW();

            int getT();

			/**
			 * \brief	Get tolerance
			 * @return	tolerance
			 */
			double getTau();

            virtual EReturn reinitialise(rapidjson::Document& document);
		protected:
			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle The handle to the XML-element describing the Problem Definition
			 * @return Indication of success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

			Eigen::MatrixXd config_w_;	//Configuration weight
			double tau_;	// Tolerance
            int T_;

	};
	typedef boost::shared_ptr<exotica::IKProblem> IKProblem_ptr;
}

#endif /* IK_PROBLEM_H_ */
