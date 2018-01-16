#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
import math
from pyexotica.publish_trajectory import *
from time import sleep

def figureEight(t):
    return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

dt=0.002
t=0.0
q=array([0.0]*7)
print('Publishing IK')
signal.signal(signal.SIGINT, sigIntHandler)
while True:
    try:
        problem.setGoal('Position',figureEight(t))
        problem.startState = q
        q = solver.solve()[0]
        publishPose(q, problem)
        sleep(dt)
        t=t+dt
    except KeyboardInterrupt:
        break

