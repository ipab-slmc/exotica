#!/usr/bin/env python
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
(sol, prob) = exo.Initializers.load_xml_full(
    '{exotica_examples}/resources/configs/time_indexed_sampling_demo_freebase.xml')
problem = exo.Setup.create_problem(prob)
solver = exo.Setup.create_solver(sol)

# set planar base limits
base_limits = np.array([[-10, 10], [-10, 10], [0, 2 * np.pi]])
problem.get_scene().get_solver().set_planar_base_limits_pos_xy_euler_z(
    base_limits[:, 0], base_limits[:, 1])

# Only specify problem _after_ setting the floating base limits (will fail otherwise!)
solver.specify_problem(problem)


solution = solver.solve()
Ts = solution[:, 0]
solution = solution[:, 1:]
# print(Ts)
# print(solution)
publish_time_indexed_trajectory(solution, Ts, problem)
