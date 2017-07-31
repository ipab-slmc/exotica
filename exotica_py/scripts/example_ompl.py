#!/usr/bin/env python

import exotica_py as exo
import rospkg
from numpy import array
from numpy import matrix

from publish_trajectory import *

def findpkg(pkg):
    return rospkg.RosPack().get_path(pkg)

(sol, prob)=exo.Initializers.loadXMLFull(findpkg('exotica_examples')+'/resources/ompl_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

solution = solver.solve(array([0.0]*7))

plot(solution)

publishTrajectory(solution, 3.0, problem)
