#!/usr/bin/env python

import pyexotica as exo
import exotica_ompl_solver_py as ompl
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import publishTrajectory

exo.Setup.initRos()
prm = exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_lazy_prm.xml')
# This LazyPRM has been setup for multi-query operation.
# You have to call prm.clearQuery() between runs.
# Call prm.clear() to discard the roadmap.

for i in range(10):
    # Solve and expand the roadmap
    solution = prm.solve()

    # Clear previous start/goal
    prm.clearQuery()

publishTrajectory(solution, 3.0, prm.getProblem())
