#!/usr/bin/env python
# coding: utf-8
import roslib
import unittest
from pyexotica.testing import check_dynamics_solver_derivatives

PKG = 'exotica_examples'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.


class TestDynamicsSolver(unittest.TestCase):
    def test_double_integrator_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/DoubleIntegratorDynamicsSolver',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.urdf',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.srdf',
                                          u'arm')

    def test_cartpole_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/CartpoleDynamicsSolver')

    def test_pendulum_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/PendulumDynamicsSolver')

    # def test_quadrotor_dynamics_solver(self):
    #     check_dynamics_solver_derivatives('exotica/QuadrotorDynamicsSolver',
    #                                       u'{exotica_examples}/resources/robots/quadrotor.urdf',
    #                                       u'{exotica_examples}/resources/robots/quadrotor.srdf',
    #                                       u'base')

    def test_pinocchio_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/PinocchioDynamicsSolver',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.urdf',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.srdf',
                                          u'arm')

    def test_pinocchio_gravity_compensation_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/PinocchioDynamicsSolverWithGravityCompensation',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.urdf',
                                          u'{exotica_examples}/resources/robots/lwr_simplified.srdf',
                                          u'arm')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestDynamicsSolver', TestDynamicsSolver)
