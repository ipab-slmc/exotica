#!/usr/bin/env python
import numpy as np
import pyexotica as exo
PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

class TestClass(unittest.TestCase):
    def setUp(self):
        (_, self.ik_prob) = exo.Initializers.load_xml_full(
            '{exotica_examples}/test/resources/test_com_valkyrie.xml')
        self.ik_problem = exo.Setup.create_problem(self.ik_prob)

    def test_zeros(self):
        # Test with zeros
        self.ik_problem.update(np.zeros(38,))
        drake_com = np.array([-0.007956278619357, -0.000791781655122, -0.023887289533801])
        exotica_com = self.ik_problem.Phi.data
        assert np.allclose(drake_com, exotica_com)

    def test_xyz_movement(self):
        # Test x,y,z movement
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 2
        test_vector[2] = 3
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([0.992043721380643, 1.999208218344877, 2.976112710466198])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_ones(self):
        # Test with ones
        drake_ones_com = np.array([1.011827488179183, 0.984257082034692, 0.909060824252459])
        self.ik_problem.update(np.ones(38,))
        exotica_ones_com = self.ik_problem.Phi.data
        assert np.allclose(drake_ones_com, exotica_ones_com)

    def test_ones_rpy_zero(self):
        # Test with ones, RPY=0
        drake_rpy_com = np.array([1.072817938570169, 0.943826381822213, 0.985867234115696])
        test_vector = np.ones(38,)
        test_vector[3] = 0
        test_vector[4] = 0
        test_vector[5] = 0
        self.ik_problem.update(test_vector)
        exotica_rpy_com = self.ik_problem.Phi.data
        assert np.allclose(drake_rpy_com, exotica_rpy_com)

    def test_roll_1(self):
        # Test Roll ALL ZEROS, xyz=1, roll=1.57
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 1
        test_vector[2] = 1
        test_vector[3] = 1.57
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([0.992043721380643, 1.023886651443021, 0.999189196509224])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_roll_2(self):
        # Test Roll ALL ZEROS, xyz=1, roll=1.57/3
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 1
        test_vector[2] = 1
        test_vector[3] = 1.57/3
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([0.992043721380643, 1.011252345052587, 0.978914122017840])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_pitch(self):
        # Test Pitch ALL ZEROS, xyz=1, pitch=1.57
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 1
        test_vector[2] = 1
        test_vector[4] = 1.57
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([0.976106382242915, 0.999208218344877, 1.007937254009971])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_yaw_1(self):
        # Test Yaw ALL ZEROS, xyz=1, yaw=1.57
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 1
        test_vector[2] = 1
        test_vector[5] = 1.57
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([1.000785445606890, 0.992043093386445, 0.976112710466199])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_yaw_2(self):
        # Test Yaw ALL ZEROS, xyz=1, roll=1.57, pitch=1.57
        test_vector = np.zeros(38,)
        test_vector[0] = 1
        test_vector[1] = 1
        test_vector[2] = 1
        test_vector[3] = 1.57
        test_vector[4] = 1.57
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([0.999182860969121, 1.023886651443021, 1.007955630432197])
        assert np.allclose(drake_test_vector, exotica_test_vector)

    def test_rpy(self):
        # Test RPY again ALL ZEROS, roll=1.57/4, pitch=-1.57/4, yaw=1.57/4
        test_vector = np.zeros(38,)
        test_vector[3] = 1.57/4
        test_vector[4] = -1.57/4
        test_vector[5] = 1.57/4
        self.ik_problem.update(test_vector)
        exotica_test_vector = self.ik_problem.Phi.data
        drake_test_vector = np.array([-0.002100122545919, 0.008227677204985, -0.023715537144087])
        assert np.allclose(drake_test_vector, exotica_test_vector)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestValkyrieCOM', TestClass)