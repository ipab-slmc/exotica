#!/usr/bin/env python

import exotica_py as exo
from numpy import array
from numpy import matrix
from publish_trajectory import *
import math

def figureEight(t):
    return array([0.0, math.sin(t * 2.0 * math.pi * 0.5) * 0.1, math.sin(t * math.pi * 0.5) * 0.2, 0.0, 0.0, 0.0])

(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/aico_solver_demo_eight.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

for t in range(0,problem.T):
    if t<problem.T/5:
        problem.setRho('Frame',0.0,t)
    else:
        problem.setRho('Frame',1e5,t)
        problem.setGoal('Frame',figureEight(t*problem.tau),t)

solution = solver.solve(array([0.0]*7))

plot(solution)

publishTrajectory(solution, problem.T*problem.tau, problem)

