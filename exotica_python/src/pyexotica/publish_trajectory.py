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

def publishPose(q, problem, t=0.0):
    problem.getScene().Update(q, t)
    problem.getScene().getSolver().publishFrames()

def publishTrajectory(traj, T, problem):
    print('Playing back trajectory '+str(T)+'s')
    dt = float(T)/float(len(traj))
    t=0
    while not is_shutdown():
        publishPose(traj[t], problem, float(t)*dt)
        sleep(dt)
        t=(t+1)%len(traj)

def publishTimeIndexedTrajectory(traj, Ts, problem):
    print('Playing back trajectory '+str(len(Ts))+' states in '+str(Ts[len(Ts)-1]))
    idx=0

    while not is_shutdown():
        for i in range(1, len(Ts)-1):
            if not is_shutdown():
                publishPose(traj[i], problem, Ts[i])
                sleep(Ts[i]-Ts[i-1])

def plot(solution):
    print('Plotting the solution')
    plt.plot(solution,'.-')
    plt.show()
