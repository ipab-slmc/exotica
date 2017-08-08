#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
import math
from pyexotica.publish_trajectory import *
from time import sleep

def figureEight(t):
    return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/ik_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

dt=0.002
t=0.0
q=array([0.0]*7)
print('Publishing IK')
while not is_shutdown():
    problem.setGoal('Position',figureEight(t))
    q = solver.solve(q)[0]
    publishPose(q, problem)    
    sleep(dt)
    t=t+dt

