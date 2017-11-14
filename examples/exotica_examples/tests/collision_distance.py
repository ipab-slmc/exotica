#!/usr/bin/env python
import pyexotica as exo
import math
import time
import numpy as np

exo.Setup.initRos()

# Primitive Box vs Primitive Box with a distance of 1.0
PrimitiveBox_vs_PrimitiveBox_Distance = ('exotica/UnconstrainedEndPoseProblem',
                                         {'Name': 'TestProblem',
                                          'PlanningScene': [('exotica/Scene',
                                                             {'CollisionScene': 'CollisionSceneFCLLatest',
                                                              'JointGroup': 'group1',
                                                              'Name': 'TestScene',
                                                              'SRDF': '{exotica_examples}/tests/resources/A_vs_B.srdf',
                                                              'SetRobotDescriptionRosParams': '1',
                                                              'URDF': '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveBox_Distance.urdf'})]})
prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveBox_Distance)
prob.update(np.zeros(prob.N,))
scene = prob.getScene()

assert(scene.isStateValid(True) == True)
print('PrimitiveBox_vs_PrimitiveBox_Distance: isStateValid(True): PASSED')

p = scene.getCollisionDistance("A", "B")
assert(len(p) == 1)
assert(np.isclose(p[0].Distance, 1.))
assert(np.isclose(p[0].Contact1, np.array([-0.5, 0, 0])).all())
assert(np.isclose(p[0].Contact2, np.array([0.5, 0, 0])).all())
assert(np.isclose(p[0].Normal1, np.array([1, 0, 0])).all())
assert(np.isclose(p[0].Normal2, np.array([-1, 0, 0])).all())
print('PrimitiveBox_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')

#------------------------------------------------------------------------------

# Primitive Box vs Primitive Box with a penetratino of 0.2
PrimitiveBox_vs_PrimitiveBox_Penetrating = ('exotica/UnconstrainedEndPoseProblem',
                                         {'Name': 'TestProblem',
                                          'PlanningScene': [('exotica/Scene',
                                                             {'CollisionScene': 'CollisionSceneFCLLatest',
                                                              'JointGroup': 'group1',
                                                              'Name': 'TestScene',
                                                              'SRDF': '{exotica_examples}/tests/resources/A_vs_B.srdf',
                                                              'SetRobotDescriptionRosParams': '1',
                                                              'URDF': '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveBox_Penetrating.urdf'})]})
prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveBox_Penetrating)
prob.update(np.zeros(prob.N,))
scene = prob.getScene()

assert(scene.isStateValid(True) == False)
print('PrimitiveBox_vs_PrimitiveBox_Penetrating: isStateValid(True): PASSED')

p = scene.getCollisionDistance("A", "B")
assert(len(p) == 1)
assert(np.isclose(p[0].Distance, -0.2))
assert(np.isclose(p[0].Contact1, np.array([0.1, 0, 0])).all())
assert(np.isclose(p[0].Contact2, np.array([-0.1, 0, 0])).all())
assert(np.isclose(p[0].Normal1, np.array([-1, 0, 0])).all())
assert(np.isclose(p[0].Normal2, np.array([1, 0, 0])).all())
print('PrimitiveBox_vs_PrimitiveBox_Penetrating: Distance and Contact Points: PASSED')


#-----------------

scene.publishProxies(p)
scene.getSolver().publishFrames()

time.sleep(1)
