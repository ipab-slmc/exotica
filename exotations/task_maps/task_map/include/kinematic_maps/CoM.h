/*
 * CoM.h
 *
 *  Created on: 21 Mar 2014
 *      Author: yimingyang
 */

#ifndef COM_H_
#define COM_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <kinematica/KinematicTree.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

namespace exotica
{
	/**
	 * @brief	Centre of mass Task Map.
	 * 			Using a short-cut to get the jacobian from kinematica, not efficient.
	 */
	class CoM: public TaskMap
	{
		public:
			/**
			 * @brief	Constructor of CoMTaskMap
			 */
			CoM();

			/**
			 * @brief	Destructor of CoMTaskMap
			 */
			virtual ~CoM();

			/**
			 * @brief	Concrete implementation of the update method
			 * @param	x	Input configuration
			 * @return	Exotica return type
			 */
            virtual EReturn update(const Eigen::VectorXd & x, const int t);

			/**
			 * @brief	Get the task space dimension
			 * @return	Exotica return type, SUCCESS if succeeded
			 */

			/**
			 * \brief Concrete implementation of the task-space size
			 */
			virtual EReturn taskSpaceDim(int & task_dim);

		protected:
			/**
			 * @brief	Concrete implementation of the initialisation method
			 * @param	handle	XML handle
			 * @return	Exotica return type
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			/**
			 * @brief	Compute the forward map (centre of mass position)
			 * @param	phi	Forward map
			 * @return	True if succeeded, false otherwise
			 */
			bool computeForwardMap(Eigen::Vector3d & phi);

			/**
			 * @brief	Compute the jacobian
			 * @param	jca	jacobian
			 * @return	True if succeeded, false otherwise
			 */
			bool computeJacobian(Eigen::MatrixXd & jac);

			/**
			 * @brief	Change end-effectors offset to centre of mass
			 * @return	True if succeeded, false otherwise
			 */
			bool changeEffToCoM();
			Eigen::VectorXd mass_;	//!< Mass of each link
			std::vector<KDL::Vector> cog_;	//!< Centre of gravity of each link
			std::vector<KDL::Frame> tip_pose_;	//!< Tip poses
			std::vector<KDL::Frame> base_pose_;	//!< Base poses
			boost::mutex lock_;	//!< For thread synchronisation
			bool initialised_;	//!< For Error checking
	};
}

#endif /* COM_H_ */
