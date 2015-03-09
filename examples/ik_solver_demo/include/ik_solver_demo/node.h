#ifndef IK_SOLVER_DEMO_NODE_H
#define IK_SOLVER_DEMO_NODE_H

#include "ik_solver/ik_solver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <exotica/Initialiser.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

class IKSolverDemoNode
{
public:
    IKSolverDemoNode();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhg_;
    std::string resource_path_;
};

#endif // IK_SOLVER_DEMO_NODE_H
