#!/usr/bin/env python

import pyexotica as exo
import exotica_ompl_solver_py as ompl
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import publishTrajectory

exo.Setup.initRos()
solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_projections.xml')

solution = solver.solve()

publishTrajectory(solution, 3.0, solver.getProblem())
