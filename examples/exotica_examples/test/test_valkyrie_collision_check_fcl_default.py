#!/usr/bin/env python
import numpy as np
import pyexotica as exo

PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

class TestClass(unittest.TestCase):
    def test_collsion(self):
        # Default FCL
        (_, ik_prob) = exo.Initializers.loadXMLFull(
            '{exotica_examples}/test/resources/test_valkyrie_collisionscene_fcl.xml')
        ik_problem = exo.Setup.createProblem(ik_prob)
        ik_problem.update(np.zeros(ik_problem.N,))
        ik_scene = ik_problem.getScene()

        # Check number of collision robot links
        assert (len(ik_scene.getCollisionRobotLinks()) == 84)

        # Check number of collision world links
        assert (len(ik_scene.getCollisionWorldLinks()) == 0)

        # Testing isStateValid(True, 0.0) - with self-collision
        assert (ik_scene.isStateValid(True, 0.0) == True)

        # Testing isStateValid(False, 0.0) - without self-collision
        assert (ik_scene.isStateValid(False, 0.0) == True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestValkyrieCollisionDefault', TestClass)
