#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt

def publishPose(q, problem):
    problem.getScene().Update(q)
    problem.getScene().getSolver().publishFrames()

def publishTrajectory(traj, T, problem):
    print('Playing back trajectory '+str(T)+'s')
    rospy.init_node('example_python_trajectory_publisher', anonymous=True)
    rate = rospy.Rate(float(len(traj))/float(T))
    t=0
    while not rospy.is_shutdown():
        publishPose(traj[t], problem)
        rate.sleep()
        t=(t+1)%len(traj)

def plot(solution):
    print('Plotting the solution')
    plt.plot(solution,'.-')
    plt.show()
