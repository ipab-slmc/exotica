#!/usr/bin/env python

import rospy

def publishTrajectory(traj, T, problem):
    rospy.init_node('example_python_trajectory_publisher', anonymous=True)
    rate = rospy.Rate(float(len(traj))/float(T))
    t=0
    while not rospy.is_shutdown():
        problem.getScene().Update(traj[t])
        problem.getScene().getSolver().publishFrames()
        rate.sleep()
        t=(t+1)%len(traj)
