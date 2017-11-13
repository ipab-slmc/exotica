#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/time_indexed_sampling_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

solution = solver.solve()

Ts = solution[:,0]
solution = solution[:,1:]
publishTimeIndexedTrajectory(solution, Ts, problem)
