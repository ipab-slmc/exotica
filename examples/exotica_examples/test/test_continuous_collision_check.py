#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import math
import time
import numpy as np

PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

def getProblemInitializer(collisionScene, URDF):
    return ('exotica/UnconstrainedEndPoseProblem',
            {'Name': 'TestProblem',
             'PlanningScene': [('exotica/Scene',
                                {'CollisionScene': collisionScene,
                                 'JointGroup': 'group1',
                                 'Name': 'TestScene',
                                 'Debug': '0',
                                 'SRDF': '{exotica_examples}/test/resources/A_vs_B.srdf',
                                 'SetRobotDescriptionRosParams': '1',
                                 'URDF': URDF})]})

class TestClass(unittest.TestCase):
    def test_collsion(self):
        collisionScene = "CollisionSceneFCLLatest"

        urdfs_to_test = ['{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveSphere_Distance.urdf', '{exotica_examples}/test/resources/Mesh_vs_Mesh_Distance.urdf']

        for urdf in urdfs_to_test:
            print("Testing", urdf)
            
            initializer = getProblemInitializer(collisionScene, urdf)
            prob = exo.Setup.create_problem(initializer)
            prob.update(np.zeros(prob.N,))
            scene = prob.get_scene()
            cs = scene.getCollisionScene()

            # Should collide at -2
            p = cs.continuousCollisionCheck(
                    "A_collision_0", exo.KDLFrame([-3., 0.0, 0.0]), exo.KDLFrame([-1.0, 0.0, 0.0]),
                    "B_collision_0", exo.KDLFrame([0, 0, 0]), exo.KDLFrame([0, 0, 0]))
            assert(p.InCollision == True)
            assert((p.TimeOfContact - 0.5) < 0.1)
            assert(np.isclose(p.ContactTransform1.getTranslation(), np.array([-2, 0, 0]), atol=0.15).all())
            assert(np.isclose(p.ContactTransform2.getTranslation(), np.array([0, 0, 0])).all())
            print(p)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestContinuousCollision', TestClass)
