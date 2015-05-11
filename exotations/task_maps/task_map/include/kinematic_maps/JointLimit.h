/*
 * JointLimit.h
 *
 *  Created on: 22 Jul 2014
 *      Author: yiming
 */

#ifndef JOINTLIMIT_H_
#define JOINTLIMIT_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

namespace exotica
{
	/**
	 * \brief	Implementation of joint limits task map.
	 * 			Note: we dont want to always stay at the centre of the joint range,
	 * 			be lazy as long as the joint is not too close to the low/high limits
	 */
	class JointLimit: public TaskMap
	{
		public:
			//	Default constructor
			JointLimit();
			//	Default destructor
			~JointLimit();

			/**
			 * @brief	Concrete implementation of update method
			 * @param	x	Joint space configuration
			 */
            virtual EReturn update(Eigen::VectorXdRefConst x, const int t);

			/**
			 * @brief	Get the task space dimension
			 * @return	Exotica return type, SUCCESS if succeeded
			 */
			virtual EReturn taskSpaceDim(int & task_dim);

		protected:
			/**
			 * @brief	Concrete implementation of initialisation from xml
			 * @param	handle	XML handler
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			Eigen::VectorXd low_limits_;	//	Lower joint limits
			Eigen::VectorXd high_limits_;	//	Higher joint limits
			Eigen::VectorXd center_;		//	Center of the joint range
			Eigen::VectorXd tau_;			//	Joint limits tolerance
			bool initialised_;

	};
}

#endif /* JOINTLIMIT_H_ */
