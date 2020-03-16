#!/usr/bin/env python
# coding: utf-8
import roslib
import unittest
from pyexotica.testing import check_dynamics_solver_derivatives

PKG = 'exotica_examples'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.

class TestDynamicsSolver(unittest.TestCase):
    def test_double_integrator_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/DoubleIntegratorDynamicsSolver', u'{exotica_examples}/resources/robots/lwr_simplified.urdf', u'{exotica_examples}/resources/robots/lwr_simplified.srdf', u'arm')

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestDynamicsSolver', TestDynamicsSolver)
