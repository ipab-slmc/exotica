#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *


(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/ompl_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

solution = solver.solve()

plot(solution)

publishTrajectory(solution, 3.0, problem)
