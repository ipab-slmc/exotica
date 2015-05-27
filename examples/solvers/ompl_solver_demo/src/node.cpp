#include "ompl_solver_demo/node.h"

using namespace exotica;

OMPLSolverDemoNode::OMPLSolverDemoNode() : nh_("~"), nhg_()
{

    {
        // Declarations
        Initialiser ini;
        std::vector<MotionSolver_ptr> sol;
        Server_ptr ser;
        std::vector<PlanningProblem_ptr> prob;

        // Get config file path, problem name and solver name from launch file
        std::string config_name;
        std::vector<std::string> problem_name(2), solver_name(1);
        nh_.getParam("config", config_name);
        nh_.getParam("problem", problem_name[0]);
        nh_.getParam("bias", problem_name[1]);
        nh_.getParam("solver", solver_name[0]);
        ROS_INFO_STREAM("Config: "<<config_name<<"\nSolver: "<<solver_name[0]<<"\nProblem: "<<problem_name[0]<<"\nBias: "<<problem_name[1]);


        // Initialise and solve
        if(ok(ini.initialise(config_name, ser, sol, prob,problem_name,solver_name)))
        {
            // Assign the problem to the solver
            // Cast the generic solver instance into IK solver
            OMPLsolver_ptr solOMPL=boost::static_pointer_cast<OMPLsolver>(sol[0]);
            if(!ok(solOMPL->specifyProblem(prob[0], NULL, prob[1], NULL)) ) {INDICATE_FAILURE; return;}
            // Create the initial configuration
            Eigen::VectorXd q=Eigen::VectorXd::Zero(prob[0]->scenes_.begin()->second->getNumJoints());
            Eigen::MatrixXd solution;

            HIGHLIGHT("Problem description:\n"<<solOMPL->print(""));

            ROS_INFO_STREAM("Calling solve()");

            ros::WallTime start_time = ros::WallTime::now();
            // Solve the problem using the IK solver
            if(ok(solOMPL->Solve(q,solution)))
            {
                double time=ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
                ROS_INFO_STREAM_THROTTLE(0.5,"Finished solving ("<<time<<"s)");
                ROS_INFO_STREAM_THROTTLE(0.5,"Solution "<<solution.row(solution.rows()-1));

                // Publish the states to rviz
                jointStatePublisher_ = nhg_.advertise<sensor_msgs::JointState>("/joint_states", 1);
                sensor_msgs::JointState jnt;
                jnt.position.resize(solution.cols());
                jnt.name = prob[0]->scenes_.begin()->second->getSolver().getJointNames();
                ros::Rate loop_rate(50.0); // Magic number for now
                int t=0;
                ROS_INFO_STREAM_THROTTLE(0.5,"Publishing states to rviz ...");
                while (ros::ok())
                {
                    jnt.header.stamp = ros::Time::now();
                    jnt.header.seq++;
                    for (int j = 0; j < solution.cols(); j++)
                        jnt.position[j] = solution(t, j);
                    jointStatePublisher_.publish(jnt);

                    t=t+1>=solution.rows()?0:t+1;
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            else
            {
                double time=ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
                ROS_INFO_STREAM_THROTTLE(0.5,"Failed to find solution ("<<time<<"s)");
            }

        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "OMPLSolverDemoNode");
    ROS_INFO_STREAM("Started");
    OMPLSolverDemoNode ex;
    ros::spin();
}
