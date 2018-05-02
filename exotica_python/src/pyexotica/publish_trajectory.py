from __future__ import print_function
from time import sleep
import matplotlib.pyplot as plt
import signal


def sigIntHandler(signal, frame):
    raise KeyboardInterrupt


def publishPose(q, problem, t=0.0):
    problem.getScene().Update(q, t)
    problem.getScene().getSolver().publishFrames()


def publishTrajectory(traj, T, problem):
    if len(traj) == 0:
        print("Trajectory has zero elements")
        raise
    signal.signal(signal.SIGINT, sigIntHandler)
    print('Playing back trajectory '+str(T)+'s')
    dt = float(T)/float(len(traj))
    t = 0
    while True:
        try:
            publishPose(traj[t], problem, float(t)*dt)
            sleep(dt)
            t = (t+1) % len(traj)
        except KeyboardInterrupt:
            return False
    return True


def publishTimeIndexedTrajectory(traj, Ts, problem, once=False):
    if len(traj) == 0:
        print("Trajectory has zero elements")
        raise
    signal.signal(signal.SIGINT, sigIntHandler)
    print('Playing back trajectory '+str(len(Ts)) +
          ' states in '+str(Ts[len(Ts)-1]))
    idx = 0

    while True:
        try:
            for i in range(1, len(Ts)-1):
                publishPose(traj[i], problem, Ts[i])
                sleep(Ts[i]-Ts[i-1])
            if once:
                break
        except KeyboardInterrupt:
            return False
    return True


def plot(solution):
    print('Plotting the solution')
    plt.plot(solution, '.-')
    plt.show()
