#include <exotica/Exotica.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace exotica;

void run()
{
    ros::NodeHandle nhg_;
    ros::NodeHandle nh_("~");

    Initializer solver, problem;

    std::string file_name, solver_name, problem_name;
    nh_.getParam("ConfigurationFile",file_name);
    nh_.getParam("Solver",solver_name);
    nh_.getParam("Problem",problem_name);

    XMLLoader::load(file_name,solver, problem, solver_name, problem_name);

    HIGHLIGHT_NAMED("XMLnode","Loaded from XML");

    // Initialize

    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);

    // Assign the problem to the solver
    any_solver->specifyProblem(any_problem);

    // If necessary, modify the problem after calling sol->specifyProblem()
    // e.g. set different rho:

    if(any_problem->type()=="exotica::UnconstrainedTimeIndexedProblem")
    {
        UnconstrainedTimeIndexedProblem_ptr problem = boost::static_pointer_cast<UnconstrainedTimeIndexedProblem>(any_problem);
        for (int t = 0; t < problem->T-1; t++)
        {
            // This sets the precision of all time steps BUT the last one to zero
            // This means we only aim to minimize the task cost in the last time step
            // The rest of the trajectory minimizes the control cost
            problem->setRho("Position",0.0,t);
        }
        Eigen::VectorXd goal(3);
        goal << 0.4, -0.1, 0.5;
        problem->setGoal("Position", goal, problem->T-1);
    }

    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(
                any_problem->getScene()->getNumJoints());
    Eigen::MatrixXd solution;
    ROS_INFO_STREAM("Calling solve()");
    {
        ros::WallTime start_time = ros::WallTime::now();
        // Solve the problem using the AICO solver
        any_solver->Solve(q, solution);
        double time = ros::Duration(
                    (ros::WallTime::now() - start_time).toSec()).toSec();
        ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving ("<<time<<"s)");
        ROS_INFO_STREAM_THROTTLE(0.5, "Solution "<<solution.row(solution.rows()-1));

        ros::Rate loop_rate(50.0);
        int t = 0;
        ROS_INFO_STREAM_THROTTLE(0.5, "Publishing states to rviz ...");
        while (ros::ok())
        {
            any_problem->getScene()->Update(solution.row(t).transpose());
            any_problem->getScene()->getSolver().publishFrames();

            t = t + 1 >= solution.rows() ? 0 : t + 1;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }


    // All classes will be destroyed at this point.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExoticaManualInitializationExampleNode");
    ROS_INFO_STREAM("Started");

    // Run demo code
    run();

    // Clean up
    // Run this only after all the exoica classes have been disposed of!
    Setup::Destroy();
}
