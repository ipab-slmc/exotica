#!/usr/bin/env python

import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/aico_effvelocity_demo.xml')
problem = solver.getProblem()

problem.setRho('Identity', 1e3, 0)
problem.setGoal('Identity', np.array([-1.57, -1.57, 0, 0, 0, 0, 0]), 0)

problem.setRho('Identity', 1e3, -1)
problem.setGoal('Identity', np.array([1.57, -1.57, 0, 0, 0, 0, 0]), -1)

for i in range(problem.T):
    problem.update(np.array([-1.57, -1.57, 0, 0, 0, 0, 0]), i)

for i in range(problem.T):
    print(i, problem.getScalarTaskCost(i))

initial_traj = np.array([-1.57, -1.57, 0, 0, 0, 0, 0]).reshape(problem.N, 1).repeat(problem.T,1)

problem.InitialTrajectory = initial_traj.T
solution = solver.solve()

# for i in range(problem.T):
#     print(problem.KinematicSolutions[i].Phi)

plot(solution)

publishTrajectory(solution, problem.T*problem.tau, problem)
