/*
 * CollisionAvoidance.h
 *
 *  Created on: 16 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_

//#define C_DEBUG
#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

#ifdef C_DEBUG
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <visualization_msgs/Marker.h>
#endif

namespace exotica
{
	class CollisionAvoidance: public TaskMap
	{
		public:
			/*
			 * \brief	Default constructor
			 */
			CollisionAvoidance();

			/*
			 * \brief	Destructor
			 */
			virtual ~CollisionAvoidance();

			/**
			 * @brief	Concrete implementation of the update method
			 * @param	x	Input configuration
			 * @return	Exotica return type
			 */
			virtual EReturn update(const Eigen::VectorXd & x, const int t);

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
			//	Safety range
			EParam<std_msgs::Float64> safe_range_;

			//	End-effector names
			std::vector<std::string> effs_;

			//	Initial end-effector offsets
			std::vector<KDL::Frame> init_offsets_;

			//	Internal kinematica solver
			kinematica::KinematicTree kin_sol_;
#ifdef C_DEBUG
			ros::Publisher state_pub_;
			ros::Publisher close_pub_;
			visualization_msgs::Marker close_;
#endif
	};
}

#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_ */
