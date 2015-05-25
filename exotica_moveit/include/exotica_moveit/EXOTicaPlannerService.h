/*
 * EXOTicaPlannerService.h
 *
 *  Created on: 19 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_
#define EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_

#include <aico/AICOsolver.h>
#include <ompl_solver/OMPLsolver.h>
#include <ik_solver/ik_solver.h>
#include <exotica/Initialiser.h>
#include <exotica_moveit/ExoticaPlanningAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

namespace exotica
{
	class EXOTicaPlannerService
	{
		public:
			EXOTicaPlannerService();
			~EXOTicaPlannerService();

			bool initialise(const std::string & config, const std::string & solver,
					const std::string & problem, const std::string & group);

			/*
			 * \brief	Solve function
			 * @param	goal	Planning goal and constraints
			 */
			bool solve(const exotica_moveit::ExoticaPlanningGoalConstPtr & goal);
			bool initialised_;
		private:
			///	The ROS node handle
			ros::NodeHandle nh_;

			///	ROS action service
			actionlib::SimpleActionServer<exotica_moveit::ExoticaPlanningAction> as_;

			///	Action result
			exotica_moveit::ExoticaPlanningResult res_;

			///	Action feedback
			exotica_moveit::ExoticaPlanningFeedback fb_;

			///	EXOTica server
			exotica::Server_ptr server_;

			///	Pointer to selected solver
			exotica::MotionSolver_ptr solver_;

			///	Pointer to selected problem
			exotica::PlanningProblem_ptr problem_;

			///	ROS service
			ros::ServiceServer service_;

			///	Moveit planning scene
			moveit_msgs::PlanningScenePtr scene_;

	};
}

#endif /* EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_ */
