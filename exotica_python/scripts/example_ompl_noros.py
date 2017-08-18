#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/ompl_solver_demo.xml')
# Set UDRF and SRDF file paths (overrides using robot_description)
exo.setModel(prob, exo.Setup.getPackagePath('exotica_examples')+'/resources/lwr_simplified.urdf', exo.Setup.getPackagePath('exotica_examples')+'/resources/lwr_simplified.srdf')

problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

solution = solver.solve()

plot(solution)
