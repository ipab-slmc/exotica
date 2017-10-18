#!/usr/bin/env python

import pyexotica as exo
solver=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example.xml')
print(solver.solve())
