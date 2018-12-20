#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *
from time import sleep
import signal

def handle_attaching(i, x, scene):
    # Update state
    scene.update(x)
    if i == 0:
        # Reset object pose
        scene.attach_object_local(
            'Item', '', exo.KDLFrame([0.61, -0.3, 0.5, 0, 0, 0, 1]))
    if i == 1:
        # Attach to end effector
        scene.attach_object('Item', 'lwr_arm_6_link')
    if i == 2:
        # Detach
        scene.detach_object('Item')


exo.Setup.init_ros()
ik = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/example_manipulate_ik.xml')
ompl = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/example_manipulate_ompl.xml')

# Plan 3 end poses
ik.get_problem().set_rho('Position1', 1e3)
ik.get_problem().set_rho('Position2', 0)
goal_pose = [ik.solve()[0].tolist()]
ik.get_problem().set_rho('Position1', 0)
ik.get_problem().set_rho('Position2', 1e3)
goal_pose.append(ik.solve()[0].tolist())
goal_pose.append([0]*len(goal_pose[0]))
start_pose = [[0]*len(goal_pose[0]), goal_pose[0], goal_pose[1]]

# Plan 3 trajectories
solution = []
for i in range(0, 3):
    print(i)
    ompl.get_problem().start_state = start_pose[i]
    ompl.get_problem().goal_state = goal_pose[i]
    handle_attaching(i, start_pose[i], ompl.get_problem().get_scene())
    solution.append(ompl.solve().tolist())

# Playback
dt = 0.03
signal.signal(signal.SIGINT, sig_int_handler)
while True:
    try:
        for i in range(0, len(solution)):
            handle_attaching(i, start_pose[i], ompl.get_problem().get_scene())
            for t in range(0, len(solution[i])):
                publish_pose(solution[i][t], ompl.get_problem())
                sleep(dt)
    except KeyboardInterrupt:
        break
