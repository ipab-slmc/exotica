#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import numpy as np
from numpy import array
from pyexotica.publish_trajectory import *

solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/ompl_solver_demo_freebase.xml')

print("joint limits:\n" +
      str(solver.get_problem().get_scene().get_kinematic_tree().get_joint_limits()))

solution = solver.solve()

plot(solution)
