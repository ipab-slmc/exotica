#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *
from numpy import array
import math

def figureEight(t):
    return array([0.0, math.sin(t * 2.0 * math.pi * 0.5) * 0.1, math.sin(t * math.pi * 0.5) * 0.2, 0.0, 0.0, 0.0])

exo.Setup.initRos()
solver=exo.Setup.loadSolver('{exotica_examples}/resources/configs/aico_solver_demo_eight.xml')
problem = solver.getProblem()

for t in range(0,problem.T):
    if t<problem.T/5:
        problem.setRho('Frame',0.0,t)
    else:
        problem.setRho('Frame',1e5,t)
        problem.setGoal('Frame',figureEight(t*problem.tau),t)

solution = solver.solve()

plot(solution)

publishTrajectory(solution, problem.T*problem.tau, problem)

