#!/usr/bin/env python

import pyexotica as exo
solver=exo.Setup.loadSolver('{exotica}/resources/configs/example.xml')
print(solver.solve())
