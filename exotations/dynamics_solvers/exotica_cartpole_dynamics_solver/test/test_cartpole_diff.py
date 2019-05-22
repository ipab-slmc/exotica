import autograd.numpy as np
from autograd import grad, jacobian, hessian
import pyexotica as exo
from time import time, sleep
import unittest
from numpy import testing as nptest
sin = np.sin
cos = np.cos

class TestCartpoleDiff(unittest.TestCase):
    """Tests cartpole derivatives against python's autograd.""" 
    @staticmethod
    def dynamics(state, u):
        # Analytical dynamics from underactuated robotics
        #   http://underactuated.mit.edu
        x, theta, xdot, thetadot = state
        l, m_c, m_p, g = 1, 1, 1, 9.81
        s, c = np.sin(theta), np.cos(theta)
        tdots = thetadot ** 2

        return np.array([
            xdot,
            thetadot,
            ((u + m_p * s * (l * (thetadot ** 2) + g * c)) /\
                (m_c + m_p * (s ** 2)))[0],
            (- (l * m_p * c * s * (thetadot ** 2) + u * c + (m_c + m_p) * g * s ) /\
                (l * m_c + l * m_p * (s ** 2)))[0]
        ])

    def setUp(self):
        # set up exotica with the test configuration
        exo.Setup.init_ros()
        sleep(0.2)

        prob = exo.Initializers.load_xml(
            '{exotica_cartpole_dynamics_solver}/test/test_cartpole_diff.xml')
        problem = exo.Setup.create_problem(prob)

        # alias derivatives and dynamic solver
        self.dynamics_solver = problem.get_scene().get_dynamics_solver()
        self.fx = jacobian(TestCartpoleDiff.dynamics, argnum=0)
        self.fu = jacobian(TestCartpoleDiff.dynamics, argnum=1)

    def test_diff(self):
        # set the seed so this is deterministic
        np.random.seed(42)

        # check against 100 state,control pairs
        for i in range(100):
            x = np.random.uniform(size=(1, 4))[0]
            u = np.array([np.random.uniform()])
            auto_diff = self.fx(x, u)
            solver_diff = self.dynamics_solver.fx(x, u)
            nptest.assert_allclose(auto_diff, solver_diff, err_msg="Derivative w.r.t. state test failed")
            
            auto_diff = self.fu(x, u)
            solver_diff = self.dynamics_solver.fu(x, u)
            nptest.assert_allclose(auto_diff, solver_diff, err_msg="Derivative w.r.t. controls test failed")


if __name__ == '__main__':
    unittest.main()
