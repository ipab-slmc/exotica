#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
import math
from pyexotica.publish_trajectory import *
from time import sleep

def com(t):
    return array([-0.1, math.sin(t * 0.5 * math.pi) * 0.3, 0.9])

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_quasistatic_valkyrie.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

tick = exo.Timer()
t=0.0;
q=problem.startState
print('Publishing IK')
signal.signal(signal.SIGINT, sigIntHandler)
pose=[0]*20;
while True:
    try:
        problem.setGoal('CoM',com(tick.getDuration()))
        pose[3]=math.sin(tick.getDuration() * 0.25 * math.pi)*0.8;
        pose[13]=-math.sin(tick.getDuration() * 0.25 * math.pi)*0.8;
        problem.setGoal('Pose',pose)
        problem.startState = q
        q = solver.solve()[0]
        publishPose(q, problem)
    except KeyboardInterrupt:
        break

