/*
 * AICOexample.cpp
 *
 *  Created on: 30 Apr 2014
 *      Author: s0972326
 */

#include "aico/AICOexample.h"

namespace exotica
{

    AICOexample::AICOexample () : nh_("~"), nhg_()
	{
		std::string resource_path = ros::package::getPath("aico").append("/resources/");
		ROS_INFO_STREAM("Loaded path: " << resource_path);

        // Declarations
        Initialiser ini;
        MotionSolver_ptr sol;
        Server_ptr ser;
        PlanningProblem_ptr prob;

        // Get config file path, problem name and solver name from launch file
        std::string problem_name, solver_name, config_name;
        nh_.getParam("config", config_name);
        nh_.getParam("problem", problem_name);
        nh_.getParam("solver", solver_name);
        ROS_INFO_STREAM("Config: "<<config_name<<"\nSolver: "<<solver_name<<"\nProblem: "<<problem_name);

        // Initialise and solve
        if(ok(ini.initialise(config_name, ser, sol, prob,problem_name,solver_name)))
        {
            // Cast the generic solver instance into AICO solver
            AICOsolver_ptr solAICO=boost::static_pointer_cast<AICOsolver>(sol);

            // Assign the problem to the solver
            if(!ok(sol->specifyProblem(prob))) {INDICATE_FAILURE; return;}

            // If necessary, modify the problem after calling sol->specifyProblem()
            // e.g. set different rho:
            for(int t=0;t<solAICO->getProblem()->getT();t++)
            {
                // This sets the precision of all time steps BUT the last one to zero
                // This means we only aim to minimize the task cost in the last time step
                // The rest of the trajectory minimizes the control cost
                solAICO->rhos.at(t).setZero();
            }

            // Create the initial configuration
            Eigen::VectorXd q=Eigen::VectorXd::Zero(prob->scenes_.begin()->second->getNumJoints());
            Eigen::MatrixXd solution;
            ROS_INFO_STREAM("Calling solve()");
            {
                ros::WallTime start_time = ros::WallTime::now();
                // Solve the problem using the AICO solver
                if(ok(solAICO->Solve(q,solution)))
                {
                    double time=ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
                    ROS_INFO_STREAM_THROTTLE(0.5,"Finished solving ("<<time<<"s)");
                    ROS_INFO_STREAM_THROTTLE(0.5,"Solution "<<solution.row(solution.rows()-1));
                    solAICO->saveCosts(std::string("costs.txt"));
                }
                else
                {
                    double time=ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
                    ROS_INFO_STREAM_THROTTLE(0.5,"Failed to find solution ("<<time<<"s)");
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
