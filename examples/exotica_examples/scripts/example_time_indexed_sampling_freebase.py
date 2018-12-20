#!/usr/bin/env python
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import *

exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/time_indexed_sampling_demo_freebase.xml')
problem = solver.get_problem()

solution = solver.solve()
Ts = solution[:, 0]
solution = solution[:, 1:]
publish_time_indexed_trajectory(solution, Ts, problem)
