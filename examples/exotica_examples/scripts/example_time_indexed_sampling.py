#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
(sol, prob) = exo.Initializers.load_xml_full(
    '{exotica_examples}/resources/configs/time_indexed_sampling_demo.xml')
problem = exo.Setup.create_problem(prob)
solver = exo.Setup.create_solver(sol)
solver.specify_problem(problem)

solution = solver.solve()
Ts = solution[:, 0]
solution = solution[:, 1:]
# print(Ts)
# print(solution)
publish_time_indexed_trajectory(solution, Ts, problem)
