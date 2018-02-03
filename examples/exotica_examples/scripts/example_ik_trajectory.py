#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from pyexotica.publish_trajectory import *
from time import sleep

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/example_ik_trajectory.xml')
print(prob)
print(sol)
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
        problem.startState = q
        problem.startTime = t
        q = solver.solve()[0]
        publishPose(q, problem, t)
        sleep(dt)
        t=(t+dt)%7.0
    except KeyboardInterrupt:
        break

