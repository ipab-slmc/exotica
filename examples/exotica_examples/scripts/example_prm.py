#!/usr/bin/env python

import pyexotica as exo
import ompl_solver_py as ompl
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
prm=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_prm.xml')

solution = prm.solve()

publishTrajectory(solution, 3.0, prm.getProblem())
