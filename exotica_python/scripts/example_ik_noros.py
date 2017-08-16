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
# Set UDRF and SRDF file paths (overrides using robot_description)
prob[1]['PlanningScene'][0][1]['URDF'] = exo.Setup.getPackagePath('exotica_examples')+'/resources/lwr_simplified.urdf'
prob[1]['PlanningScene'][0][1]['SRDF'] = exo.Setup.getPackagePath('exotica_examples')+'/resources/lwr_simplified.srdf'

problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

dt=1.0/20.0
t=0.0
q=array([0.0]*7)
print('Publishing IK')
while True:
    problem.setGoal('Position',figureEight(t))
    problem.startState = q
    q = solver.solve()[0]
    print(q)
    sleep(dt)
    t=t+dt

