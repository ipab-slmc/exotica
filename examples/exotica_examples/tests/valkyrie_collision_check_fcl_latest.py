#!/usr/bin/env python
import numpy as np
import pyexotica as exo

print(">>>>>>>>>>>>> TESTING FCL LATEST")

# FCL Latest
(_, ik_prob) = exo.Initializers.loadXMLFull(
    '{exotica_examples}/resources/configs/test_valkyrie_collisionscene_fcl_latest.xml')
ik_problem = exo.Setup.createProblem(ik_prob)
ik_problem.update(np.zeros(ik_problem.N,))
ik_scene = ik_problem.getScene()

print("Check number of collision robot links")
assert (len(ik_scene.getCollisionRobotLinks()) == 84)

print("Check number of collision world links")
assert (len(ik_scene.getCollisionWorldLinks()) == 0)

print("Testing isStateValid(True, 0.0) - with self-collision")
assert (ik_scene.isStateValid(True, 0.0) == True)

print("Testing isStateValid(False, 0.0) - without self-collision")
assert (ik_scene.isStateValid(False, 0.0) == True)

print('>>SUCCESS<<')
