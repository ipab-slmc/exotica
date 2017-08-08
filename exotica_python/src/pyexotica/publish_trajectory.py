#!/usr/bin/env python

from time import sleep
import matplotlib.pyplot as plt
import signal

trajectoryPlaybackIsShutdown = False

def sigIntHandler(signal, frame):
    global trajectoryPlaybackIsShutdown
    trajectoryPlaybackIsShutdown = True
    raise KeyboardInterrupt

def is_shutdown():
    signal.signal(signal.SIGINT, sigIntHandler)
    global trajectoryPlaybackIsShutdown
    return trajectoryPlaybackIsShutdown

def publishPose(q, problem):
    problem.getScene().Update(q)
    problem.getScene().getSolver().publishFrames()

def publishTrajectory(traj, T, problem):
    print('Playing back trajectory '+str(T)+'s')
    dt = float(T)/float(len(traj))
    t=0
    while not is_shutdown():
        publishPose(traj[t], problem)
        sleep(dt)
        t=(t+1)%len(traj)

def plot(solution):
    print('Plotting the solution')
    plt.plot(solution,'.-')
    plt.show()
