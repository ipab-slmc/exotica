#!/usr/bin/env python
print('Start')
import pyexotica as exo
from numpy import array
from numpy import matrix
import math
from time import sleep

def figureEight(t):
    return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/ik_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

dt=1.0/20.0
t=0.0
q=array([0.0]*7)
print('Publishing IK')
timer = exo.Timer()
while True:
    timer.reset()
    problem.setGoal('Position',figureEight(t))
    problem.startState = q
    q = solver.solve()[0]
    print('Solution found in '+str(timer.getDuration())+'s '+str(q))
    sleep(dt)
    t=t+dt

