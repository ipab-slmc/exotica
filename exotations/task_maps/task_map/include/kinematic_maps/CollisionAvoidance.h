/*
 * CollisionAvoidance.h
 *
 *  Created on: 16 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_

#define C_DEBUG
#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

#ifdef C_DEBUG
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
			virtual EReturn update(Eigen::VectorXdRefConst x, const int t);

			/**
			 * \brief Concrete implementation of the task-space size
			 */
			virtual EReturn taskSpaceDim(int & task_dim);

            EReturn setPreUpdateCallback(boost::function<void(CollisionAvoidance*, Eigen::VectorXdRefConst,int)> pre_update_callback);
            EReturn setObsFrame(const KDL::Frame & tf);

		protected:
			/**
			 * @brief	Concrete implementation of the initialisation method
			 * @param	handle	XML handle
			 * @return	Exotica return type
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			//	Indicate if self collision checking is required
			EParam<std_msgs::Bool> self_;

			//	Safety range
			EParam<std_msgs::Float64> safe_range_;

			//	End-effector names
			std::vector<std::string> effs_;

			//	Initial end-effector offsets
			std::vector<KDL::Frame> init_offsets_;

			//	Internal kinematica solver
			kinematica::KinematicTree kin_sol_;

            boost::function<void(CollisionAvoidance*, Eigen::VectorXdRefConst, int)> pre_update_callback_;
            fcl::Transform3f obs_in_base_tf_;

#ifdef C_DEBUG
			ros::Publisher close_pub_;
			ros::Publisher robot_centre_pub_;
			ros::Publisher world_centre_pub_;

			ros::NodeHandle nh_;
			visualization_msgs::Marker close_;
			visualization_msgs::Marker robot_centre_;
			visualization_msgs::Marker world_centre_;
#endif
	};
}

#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_COLLISIONAVOIDANCE_H_ */
