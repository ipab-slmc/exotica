/*
 * JointSpaceSampling.h
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_JOINTSPACESAMPLING_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_JOINTSPACESAMPLING_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>

namespace exotica
{
	class JointSpaceSampling: public TaskMap
	{
		public:
			/*
			 * \brief	Default constructor
			 */
			JointSpaceSampling();

			/*
			 * \brief	Default destructor
			 */
			virtual ~JointSpaceSampling();

			/**
			 * @brief	Concrete implementation of update method
			 * @param	x	Joint space configuration
			 */
			virtual EReturn update(const Eigen::VectorXd & x, const int t);

			/*
			 * \brief	Check state validation
			 * @param	valid	Validation flag
			 */
			EReturn isStateValid(bool valid);
		protected:
			/**
			 * @brief	Concrete implementation of initialisation from xml
			 * @param	handle	XML handler
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
		private:
			bool stateValid_;	//	State validation flag
	};
}

#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_JOINTSPACESAMPLING_H_ */
