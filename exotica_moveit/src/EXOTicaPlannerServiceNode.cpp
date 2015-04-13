/*
 * EXOTicaPlannerServiceNode.cpp
 *
 *  Created on: 23 Mar 2015
 *      Author: yiming
 */

#include <ros/ros.h>
#include "exotica_moveit/EXOTicaPlannerService.h"
#include <exotica_moveit/ExoticaPlanning.h>
#include <exotica/EXOTica.hpp>

class ExoticaService
{
	public:

		ExoticaService()
		{
			//TODO
		}
		~ExoticaService()
		{
			//TODO
		}
		bool solve(exotica_moveit::ExoticaPlanning::Request & req,
				exotica_moveit::ExoticaPlanning::Response & res)
		{
			ROS_INFO("Planning request received");

			double tau_;
			bool found_solution = false;
			Eigen::MatrixXd solution;
			exotica::Initialiser ini;

			if (exotica::ok(ini.initialise(req.xml_file_, server, solver, problem)))
			{
				if (solver->type().compare("exotica::AICOsolver") == 0)
				{
					tau_ = boost::static_pointer_cast<exotica::AICOProblem>(problem)->getTau();
				}
				else if (solver->type().compare("exotica::IKsolver") == 0)
				{
					tau_ = boost::static_pointer_cast<exotica::IKProblem>(problem)->getTau();
				}
				if (!exotica::ok(solver->specifyProblem(problem)))
				{
					INDICATE_FAILURE
					return false;
				}
				moveit_msgs::PlanningScenePtr scene_ptr;
				scene_ptr.reset(new moveit_msgs::PlanningScene(req.scene_));
				if (!exotica::ok(problem->setScene(scene_ptr)))
				{
					INDICATE_FAILURE
					return false;
				}

				Eigen::VectorXd q0;
				exotica::vectorExoticaToEigen(req.q0, q0);
				ros::Time start = ros::Time::now();
				if (solver->type().compare("exotica::AICOsolver") == 0)
				{
					found_solution =
							exotica::ok(boost::static_pointer_cast<exotica::AICOsolver>(solver)->Solve(q0, solution));
				}
				else if (solver->type().compare("exotica::IKsolver") == 0)
				{
					found_solution =
							exotica::ok(boost::static_pointer_cast<exotica::IKsolver>(solver)->Solve(q0, solution));
				}

				if (found_solution)
				{
					res.succeeded_ = true;
					res.planning_time_ = ros::Duration(ros::Time::now() - start).toSec();
					exotica::matrixEigenToExotica(solution, res.solution_);
					ROS_INFO_STREAM("Solution Found in "<<res.planning_time_<<" sec");
				}
				else
				{
					ROS_ERROR("Solution Not Found");
				}
			}
			else
			{
				ROS_ERROR("EXOTica Initialisation Failed");
			}
			if (found_solution)
				return true;
			else
				return false;
		}

	private:
		exotica::Server_ptr server;
		exotica::MotionSolver_ptr solver;
		exotica::PlanningProblem_ptr problem;
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "exotica_planning_server");
	ros::NodeHandle n;
	ExoticaService ser;
	ros::ServiceServer service =
			n.advertiseService("exotica_planning", &ExoticaService::solve, &ser);
	ROS_INFO("Exotica Planning Service Is Ready");
	ros::spin();
	ros::waitForShutdown();
	ROS_INFO("Shutting Down Exotica Planning Service");
	return 0;
}
