#!/usr/bin/env python

import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/aico_effvelocity_demo.xml')
problem = solver.get_problem()

problem.set_rho('Identity', 1e3, 0)
problem.set_goal('Identity', np.array([-1.57, -1.57, 0, 0, 0, 0, 0]), 0)

problem.set_rho('Identity', 1e3, -1)
problem.set_goal('Identity', np.array([1.57, -1.57, 0, 0, 0, 0, 0]), -1)

for i in range(problem.T):
    problem.update(np.array([-1.57, -1.57, 0, 0, 0, 0, 0]), i)

for i in range(problem.T):
    print(i, problem.get_scalar_task_cost(i))

initial_traj = np.array([-1.57, -1.57, 0, 0, 0, 0, 0]
                        ).reshape(problem.N, 1).repeat(problem.T, 1)

problem.initial_trajectory = initial_traj.T
solution = solver.solve()

# for i in range(problem.T):
#     print(problem.KinematicSolutions[i].Phi)

plot(solution)

publish_trajectory(solution, problem.T * problem.tau, problem)
