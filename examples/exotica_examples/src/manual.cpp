#include <exotica/Exotica.h>

// Manual initialization requires dependency on specific solvers and task maps:
#include <ik_solver/IKsolverInitializer.h>
#include <task_map/EffPositionInitializer.h>

using namespace exotica;

void run()
{
    ros::NodeHandle nhg_;

    // Scene using joint group 'arm'
    SceneInitializer scene("MyScene","arm");
    // End-effector task map with two position frames
    EffPositionInitializer map("Position","MyScene",false,
      {FrameInitializer("lwr_arm_6_link")});
    // Create a task using the map above (goal will be specified later)
    Eigen::VectorXd W(7);
    W << 7,6,5,4,3,2,1;

    UnconstrainedEndPoseProblemInitializer problem("MyProblem",scene,W,false,{map});
    IKsolverInitializer solver("MySolver");
    solver.C = 1e-3;
    solver.MaxIt = 1;
    solver.MaxStep = 0.1;

    HIGHLIGHT_NAMED("ManualLoader","Loaded from a hardcoded specialized initializer.");

    // Initialize

    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);

    // Assign the problem to the solver
    any_solver->specifyProblem(any_problem);
    UnconstrainedEndPoseProblem_ptr my_problem = boost::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);


    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->scene_->getNumJoints());
    Eigen::MatrixXd solution;


    ROS_INFO_STREAM("Calling solve() in an infinite loop");

    double t = 0.0;
    ros::Rate loop_rate(500.0);
    ros::WallTime init_time = ros::WallTime::now();

    while (ros::ok())
    {
      ros::WallTime start_time = ros::WallTime::now();

      // Update the goal if necessary
      // e.g. figure eight
      t = ros::Duration((ros::WallTime::now() - init_time).toSec()).toSec();
      my_problem->y << 0.6,
              -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
              0.5 + sin(t * M_PI * 0.5) * 0.2;

      // Solve the problem using the IK solver
      try
      {
        any_solver->Solve(q, solution);
      }
      catch (SolveException e)
      {
        // Ignore failures
      }
      double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
      ROS_INFO_STREAM_THROTTLE(0.5,
        "Finished solving in "<<time<<"s. Solution ["<<solution<<"]");
      q = solution.row(solution.rows() - 1);

      my_problem->Update(q);
      my_problem->getScene()->getSolver().publishFrames();

      ros::spinOnce();
      loop_rate.sleep();
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
