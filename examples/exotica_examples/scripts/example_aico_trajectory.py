#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
solver=exo.Setup.loadSolver('{exotica_examples}/resources/configs/aico_trajectory.xml')
problem = solver.getProblem()

for t in range(0,problem.T):
    if float(t)*problem.tau<0.8:
        problem.setRho('FrameB',0.0,t)
    else:
        problem.setRho('FrameB',1e5,t)

solution = solver.solve()

plot(solution)

publishTrajectory(solution, problem.T*problem.tau, problem)

