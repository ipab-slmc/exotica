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

		ExoticaService() :
				nh_("~")
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
			ROS_INFO_STREAM("Planning request received for Planning group "<<req.group_name_);

			double tau_;
			bool found_solution = false;
			Eigen::MatrixXd solution;
			exotica::Initialiser ini;

			if (exotica::ok(ini.initialise(req.xml_file_, server, solver, problem)))
			{
				moveit_msgs::PlanningScenePtr scene_ptr;
				scene_ptr.reset(new moveit_msgs::PlanningScene(req.scene_));
				if (!exotica::ok(problem->setScene(scene_ptr)))
				{
					INDICATE_FAILURE
					return false;
				}
				if (solver->type().compare("exotica::AICOsolver") == 0)
				{
					tau_ = boost::static_pointer_cast<exotica::AICOProblem>(problem)->getTau();
				}
				else if (solver->type().compare("exotica::IKsolver") == 0)
				{
					tau_ = boost::static_pointer_cast<exotica::IKProblem>(problem)->getTau();
				}
				else if (solver->type().compare("exotica::OMPLsolver") == 0)
				{
					const moveit::core::JointModelGroup* model_group =
							problem->scenes_.begin()->second->getPlanningScene()->getRobotModel()->getJointModelGroup(req.group_name_);
					moveit::core::JointBoundsVector b = model_group->getActiveJointModelsBounds();
					exotica::OMPLProblem_ptr tmp =
							boost::static_pointer_cast<exotica::OMPLProblem>(problem);
					tmp->getBounds().resize(b.size() * 2);
					for (int i = 0; i < b.size(); i++)
					{
						tmp->getBounds()[i] = (*b[i])[0].min_position_;
						tmp->getBounds()[i + b.size()] = (*b[i])[0].max_position_;
					}
					exotica::OMPLsolver_ptr ss =
							boost::static_pointer_cast<exotica::OMPLsolver>(solver);
					ss->setMaxPlanningTime(req.max_time_);
				}
				if (!exotica::ok(solver->specifyProblem(problem)))
				{
					INDICATE_FAILURE
					return false;
				}

				Eigen::VectorXd q0;
				exotica::vectorExoticaToEigen(req.q0, q0);
				CHECK_EXECUTION
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
				else if (solver->type().compare("exotica::OMPLsolver") == 0)
				{
					found_solution =
							exotica::ok(boost::static_pointer_cast<exotica::OMPLsolver>(solver)->Solve(q0, solution));
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
		ros::NodeHandle nh_;
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
