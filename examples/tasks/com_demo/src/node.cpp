#include "com_demo/node.h"

using namespace exotica;

IKSolverDemoNode::IKSolverDemoNode()
    : nh_("~"), nhg_()
{

  {
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
    ROS_INFO_STREAM(
        "Config: "<<config_name<<"\nSolver: "<<solver_name<<"\nProblem: "<<problem_name);

    // Initialise and solve
    if (ok(
        ini.initialise(config_name, ser, sol, prob, problem_name, solver_name)))
    {
      // Assign the problem to the solver
      if (!ok(sol->specifyProblem(prob)))
      {
        INDICATE_FAILURE
        ;
        return;
      }
      // Create the initial configuration
      Eigen::VectorXd q = Eigen::VectorXd::Zero(
          prob->scenes_.begin()->second->getNumJoints());
      Eigen::MatrixXd solution;
      // Cast the generic solver instance into IK solver
      exotica::IKsolver_ptr solIK =
          boost::static_pointer_cast<exotica::IKsolver>(sol);
      ROS_INFO_STREAM("Calling solve() in an infinite loop");

      // Publish the states to rviz
      jointStatePublisher_ = nhg_.advertise<sensor_msgs::JointState>(
          "/joint_states", 1);
      sensor_msgs::JointState jnt;
      jnt.name = prob->scenes_.begin()->second->getSolver().getJointNames();
      jnt.position.resize(jnt.name.size());
      double t = 0.0;

      while (ros::ok())
      {
        ros::WallTime start_time = ros::WallTime::now();

        // Update the goal if necessary
        // e.g. figure eight
        Eigen::VectorXd goal(3);
        goal << 0.0, 0.2 + sin(t * 2.0 * M_PI * 0.5) * 0.05, 0.0;
        solIK->setGoal("FootPosition", goal, 0);

        // Solve the problem using the IK solver
        if (ok(solIK->Solve(q, solution)))
        {
          double time = ros::Duration(
              (ros::WallTime::now() - start_time).toSec()).toSec();
          ROS_INFO_STREAM_THROTTLE(1.0,
              "Finished solving ("<<time<<"s), error: "<<solIK->error);
          q = solution.row(solution.rows() - 1);

          jnt.header.stamp = ros::Time::now();
          jnt.header.seq++;
          for (int j = 0; j < solution.cols(); j++)
            jnt.position[j] = q(j);
          jointStatePublisher_.publish(jnt);

          ros::spinOnce();
          time =
              ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
          if (time < 0.005)
          {
            ros::Rate loop_rate(1.0 / (0.005 - time));
            loop_rate.sleep();
            t += 0.005;
          }
          else
          {
            t += time;
          }
        }
        else
        {
          double time = ros::Duration(
              (ros::WallTime::now() - start_time).toSec()).toSec();
          ROS_INFO_STREAM_THROTTLE(0.5,
              "Failed to find solution ("<<time<<"s)");
          break;
        }
      }

    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "IKSolverDemoNode");
  ROS_INFO_STREAM("Started");
  IKSolverDemoNode ex;
  ros::spin();
}
