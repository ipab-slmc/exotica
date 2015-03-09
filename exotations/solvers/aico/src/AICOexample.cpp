/*
 * AICOexample.cpp
 *
 *  Created on: 30 Apr 2014
 *      Author: s0972326
 */

#include "aico/AICOexample.h"

namespace exotica
{

	AICOexample::AICOexample ()
	{
		// TODO Auto-generated constructor stub
		std::string resource_path = ros::package::getPath("aico").append("/resources/");
		ROS_INFO_STREAM("Loaded path: " << resource_path);

		// Use Task Map registrar
		//exotica::EffPosition e;

		{
			Initialiser ini;
			MotionSolver_ptr tmp_sol;
			Server_ptr ser;
			PlanningProblem_ptr tmp_prob;
			std::vector<std::string> impl;
			MotionSolver_fac::Instance().listImplementations(impl);
			for(int i=0;i<impl.size();i++) ROS_INFO_STREAM("Solver: '"<<impl[i]<<"'");
			if(ok(ini.initialise(resource_path + std::string("example.xml"), ser, tmp_sol, tmp_prob)))
			{
				exotica::AICOsolver_ptr sol=boost::static_pointer_cast<AICOsolver>(tmp_sol);
				if(!ok(sol->specifyProblem(tmp_prob))) {INDICATE_FAILURE; return;}
				Eigen::VectorXd q0=Eigen::VectorXd::Zero(9);
				Eigen::MatrixXd solution;
				ROS_INFO_STREAM("Calling solve()");
				{
					ros::WallTime start_time = ros::WallTime::now();
					if(ok(sol->Solve(q0,solution)))
					{
						double time=ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
						//ROS_INFO_STREAM("Finished solving\nSolution:\n"<<solution);
						ROS_INFO_STREAM("Finished solving ("<<time<<"s)");
						sol->saveCosts(resource_path + std::string("costs.txt"));
					}
					else
					{
						ROS_INFO_STREAM("Failed to find solution");
					}
				}
			}
		}
	}

	AICOexample::~AICOexample ()
	{
		// TODO Auto-generated destructor stub
	}



} /* namespace exotica */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AICOexample");
	ROS_INFO_STREAM("Started");
	exotica::AICOexample ex;
	ros::spin();
}
