#!/usr/bin/env python
print('Importing pyexotica...')
import pyexotica as exo
print('OK')

print('Instanciating Setup...')
exo.Setup()
print('OK')

print('Getting solvers...')
print(exo.Setup().getSolvers())
print('OK')

print('Getting problems...')
print(exo.Setup().getProblems())
print('OK')

print('Getting maps...')
print(exo.Setup().getMaps())
print('OK')

print('Printing all classes...')
exo.Setup().printSupportedClasses()
print('OK')

print('Getting initializers...')
exo.Setup().getInitializers()
print('OK')

print('Getting package path...')
print(exo.Setup().getPackagePath('exotica_python'))
print('OK')

print('Initializing ROS node...')
exo.Setup().initRos()
print('OK')

print('Loading from XML...')
(sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica')+'/resources/configs/ik_solver_demo.xml')
print('OK')

print('Loading from XML parsed path...')
(sol, prob)=exo.Initializers.loadXMLFull('{exotica}/resources/configs/ik_solver_demo.xml')
print('OK')

print('Creating problem...')
problem = exo.Setup.createProblem(prob)
print('OK')

print('Creating solver...')
solver = exo.Setup.createSolver(sol)
print('OK')

print('Specifying problem...')
solver.specifyProblem(problem)
print('OK')

print('Solving...')
solver.solve()
print('OK')

print('>>SUCCESS<<')
