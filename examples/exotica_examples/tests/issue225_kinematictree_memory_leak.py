#!/usr/bin/env python
from __future__ import print_function
import time
import resource
import numpy as np
import pyexotica as exo

print(">>>>>>>>>>>>> Issue #225 Memory Leak")

ompl = exo.Setup.loadSolver(
    '{exotica_examples}/resources/configs/example_distance.xml')
sc = ompl.getProblem().getScene()

s = time.time()
ompl.getProblem().update(np.zeros(7,))
e = time.time()
memory_usage_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
print(">>> Before:", e-s, " - Memory Usage Before:", memory_usage_before)

print(">>> Loading and cleaning scene 500 times")
for _ in xrange(500):
    ompl.getProblem().update(np.zeros(7,))
    sc.cleanScene()
    sc.loadSceneFile(
        '{exotica_examples}/resources/scenes/example_manipulate.scene')

memory_usage_intermediate = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
print(">>> Intermediate:", " - Memory Usage Intermediate:",
      memory_usage_intermediate)

print(">>> LEAK:", memory_usage_intermediate - memory_usage_before)
assert (memory_usage_intermediate - memory_usage_before) == 0

print(">>> updateSceneFrames 10000 times")
for _ in xrange(10000):
    sc.updateSceneFrames()
    sc.updateCollisionObjects()

s = time.time()
ompl.getProblem().update(np.ones(7,))
e = time.time()
memory_usage_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
print(">>> After:", e-s, " - Memory Usage After:", memory_usage_after)

print(">>> LEAK:", memory_usage_after - memory_usage_before)
assert (memory_usage_after - memory_usage_before) == 0

print('>>SUCCESS<<')
