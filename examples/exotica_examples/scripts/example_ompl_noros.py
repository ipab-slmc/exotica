#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/ompl_solver_demo.xml')

solution = solver.solve()

plot(solution)
