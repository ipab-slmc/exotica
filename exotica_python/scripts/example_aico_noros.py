#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *

solver=exo.Setup.loadSolver('{exotica}/resources/configs/aico_solver_demo_eight.xml')
problem = solver.getProblem()

for t in range(0,problem.T):
    if float(t)*problem.tau<0.8:
        problem.setRho('Frame',0.0,t)
    else:
        problem.setRho('Frame',1e5,t)

solution = solver.solve()

plot(solution)


