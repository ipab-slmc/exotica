#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *
import signal
from time import sleep


def publish_pose(q, problem, t=0):
    '''Custom pose publishing to force update the task map debug marker publishing'''
    problem.update(q, t)
    problem.get_scene().get_solver().publish_frames()


def publish_trajectory(traj, T, problem):
    '''Custom trajectory publishing to force update the task map debug marker publishing'''
    if len(traj) == 0:
        print("Trajectory has zero elements")
        raise
    signal.signal(signal.SIGINT, sig_int_handler)
    print('Playing back trajectory '+str(T)+'s')
    dt = float(T)/float(len(traj))
    t = 0
    while True:
        try:
            publish_pose(traj[t], problem, t)
            sleep(dt)
            t = (t+1) % len(traj)
        except KeyboardInterrupt:
            return False


exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/sphere_collision.xml')
problem = solver.get_problem()

for t in range(0, problem.T-1):
    problem.set_rho('Pose', 0.0, t)

solution = solver.solve()

publish_trajectory(solution, problem.T*problem.tau, problem)
