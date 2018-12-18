#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/aico_trajectory.xml')
problem = solver.get_problem()

for t in range(0, problem.T):
    if float(t)*problem.tau < 0.8:
        problem.set_rho('Frame', 0.0, t)
    else:
        problem.set_rho('Frame', 1e5, t)

solution = solver.solve()

plot(solution)

publish_trajectory(solution, problem.T*problem.tau, problem)
