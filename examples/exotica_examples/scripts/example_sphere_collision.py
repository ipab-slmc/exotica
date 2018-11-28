#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *

# Custom pose and trajectory publishing to force update the task map debug marker publishing
def publishPose(q, problem, t=0):
    problem.update(q, t)
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
            publishPose(traj[t], problem, t)
            sleep(dt)
            t = (t+1) % len(traj)
        except KeyboardInterrupt:
            return False


exo.Setup.initRos()
solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/sphere_collision.xml')
problem = solver.getProblem()

for t in range(0, problem.T-1):
    problem.setRho('Pose', 0.0, t)

solution = solver.solve()

publishTrajectory(solution, problem.T*problem.tau, problem)
