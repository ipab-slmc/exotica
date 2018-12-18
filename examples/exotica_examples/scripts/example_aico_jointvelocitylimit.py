#!/usr/bin/env python

import signal
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import plot, publish_trajectory, sig_int_handler

if __name__ == "__main__":
    exo.Setup.init_ros()
    signal.signal(signal.SIGINT, sig_int_handler)

    # AICO requires the initial pose to be available, i.e., run IK
    ik = exo.Setup.load_solver('{exotica_examples}/resources/configs/example_manipulate_ik.xml')
    ik.get_problem().set_rho('Position1', 1e3)
    ik.get_problem().set_rho('Position2', 0)
    q_ik = ik.solve()[0].tolist()

    solver = exo.Setup.load_solver('{exotica_examples}/resources/configs/aico_jointvelocitylimit_demo.xml')
    problem = solver.get_problem()
    problem.start_state = q_ik

    problem.set_rho('Position1', 1e5, 0)
    problem.set_rho('Position2', 1e5, -1)
    solution = solver.solve()

    # Plot velocities
    plot(np.diff(solution, axis=0))

    # Plot cost evolution
    plot(problem.get_cost_evolution()[1])

    # Publish trajectory
    publish_trajectory(solution, problem.T * problem.tau, problem)
