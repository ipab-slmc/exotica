#!/usr/bin/env python

import pyexotica as exo
import ompl_solver_py as ompl
import numpy as np
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
prm=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_prm.xml')
# This PRM has been setup for multi-query operation.
# You have to call prm.clearQuery() between runs.
# Call prm.clear() to discard the roadmap.

# Grow roadmap for 1s
prm.growRoadmap(1)

solution = None
for i in xrange(10):
    # Solve
    solution = prm.solve()

    # Grow the roadmap some more
    prm.growRoadmap(1)
    # Clear previous start/goal
    prm.clearQuery()

publishTrajectory(solution, 3.0, prm.getProblem())
