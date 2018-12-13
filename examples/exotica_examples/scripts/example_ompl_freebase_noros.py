#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import numpy as np
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/ompl_solver_demo_freebase.xml')

# set planar base limits
base_limits = np.array([[-10, 10], [-10, 10], [0, 2 * np.pi]])
solver.getProblem().getScene().getSolver().setPlanarBaseLimitsPosXYEulerZ(base_limits[:,0], base_limits[:,1])

print("joint limits:\n"+str(solver.getProblem().getScene().getSolver().getJointLimits()))

solution = solver.solve()

plot(solution)
