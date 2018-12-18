#!/usr/bin/env python

import math
from time import sleep
import signal

import pyexotica as exo
from numpy import array, matrix
from pyexotica.publish_trajectory import *


def com(t):
    return array([-0.1, math.sin(t * 0.5 * math.pi) * 0.3, 0.9])


exo.Setup.init_ros()
(sol, prob) = exo.Initializers.load_xml_full(
    '{exotica_examples}/resources/configs/ik_quasistatic_valkyrie.xml')
problem = exo.Setup.create_problem(prob)
solver = exo.Setup.create_solver(sol)
solver.specify_problem(problem)

tick = exo.Timer()
t = 0.0
q = problem.start_state
print('Publishing IK')
signal.signal(signal.SIGINT, sig_int_handler)
pose = [0] * 20
stability = problem.get_task_maps()['Stability']

while True:
    try:
        problem.set_goal('CoM', com(tick.get_duration()))
        pose[3] = math.sin(tick.get_duration() * 0.25 * math.pi) * 0.8
        pose[13] = -math.sin(tick.get_duration() * 0.25 * math.pi) * 0.8
        problem.set_goal('Pose', pose)
        problem.start_state = q
        stability.debug_mode = False
        q = solver.solve()[0]
        stability.debug_mode = True
        problem.update(q)
        publish_pose(q, problem)
    except KeyboardInterrupt:
        break
