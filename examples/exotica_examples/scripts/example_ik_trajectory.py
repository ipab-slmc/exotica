#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from pyexotica.publish_trajectory import *
from time import sleep
import signal

exo.Setup.init_ros()
(sol, prob) = exo.Initializers.load_xml_full(
    '{exotica_examples}/resources/configs/example_ik_trajectory.xml')
print(prob)
print(sol)
problem = exo.Setup.create_problem(prob)
solver = exo.Setup.create_solver(sol)
solver.specify_problem(problem)

dt = 0.002
t = 0.0
q = array([0.0]*7)
print('Publishing IK')
signal.signal(signal.SIGINT, sig_int_handler)
while True:
    try:
        problem.start_state = q
        problem.start_time = t
        q = solver.solve()[0]
        publish_pose(q, problem, t)
        sleep(dt)
        t = (t+dt) % 7.0
    except KeyboardInterrupt:
        break
