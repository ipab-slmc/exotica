#!/usr/bin/env python

import signal
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import plot, publishTrajectory, sigIntHandler

if __name__ == "__main__":
    exo.Setup.initRos()
    signal.signal(signal.SIGINT, sigIntHandler)

    # AICO requires the initial pose to be available, i.e., run IK
    ik = exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_manipulate_ik.xml')
    ik.getProblem().setRho('Position1', 1e3)
    ik.getProblem().setRho('Position2', 0)
    q_ik = ik.solve()[0].tolist()

    solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/aico_jointvelocitylimit_demo.xml')
    problem = solver.getProblem()
    problem.startState = q_ik

    problem.setRho('Position1', 1e5, 0)
    problem.setRho('Position2', 1e5, -1)
    solution = solver.solve()

    # Plot velocities
    plot(np.diff(solution, axis=0))

    # Plot cost evolution
    plot(problem.getCostEvolution()[1])

    # Publish trajectory
    publishTrajectory(solution, problem.T*problem.tau, problem)
