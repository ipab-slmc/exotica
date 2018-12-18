#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import numpy as np
from numpy import array
from pyexotica.publish_trajectory import *

solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/ompl_solver_demo_freebase.xml')

# set planar base limits
base_limits = np.array([[-10, 10], [-10, 10], [0, 2 * np.pi]])
solver.get_problem().get_scene().get_solver().set_planar_base_limits_pos_xy_euler_z(base_limits[:, 0], base_limits[:, 1])

print("joint limits:\n" +
      str(solver.get_problem().get_scene().get_solver().get_joint_limits()))

solution = solver.solve()

plot(solution)
