#!/usr/bin/env python

import pyexotica as exo
solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/example.xml')
print(solver.solve())
