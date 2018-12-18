#!/usr/bin/env python
from __future__ import print_function
import signal
import pyexotica as exo
from pyexotica.publish_trajectory import sig_int_handler
from numpy import array
import math
from time import sleep


def figure_eight(t):
    return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])


(sol, prob) = exo.Initializers.load_xml_full(
    '{exotica_examples}/resources/configs/ik_solver_demo.xml')
problem = exo.Setup.create_problem(prob)
solver = exo.Setup.create_solver(sol)
solver.specify_problem(problem)

dt = 1.0/20.0
t = 0.0
q = array([0.0]*7)
print('Publishing IK')
timer = exo.Timer()
signal.signal(signal.SIGINT, sig_int_handler)
while True:
    try:
        timer.reset()
        problem.set_goal('Position', figure_eight(t))
        problem.start_state = q
        q = solver.solve()[0]
        print('Solution found in '+str(timer.get_duration())+'s '+str(q))
        sleep(dt)
        t = t+dt
    except KeyboardInterrupt:
        break
