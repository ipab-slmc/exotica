# import autograd.numpy as np
# from autograd import grad, jacobian, hessian
import numpy as np
import pyexotica as exo
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
        _, theta, xdot, thetadot = state
        l, m_c, m_p, g = 1, 1, 1, 9.81
        s, c = np.sin(theta), np.cos(theta)

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
        problem = exo.Setup.load_problem('{exotica_cartpole_dynamics_solver}/test/test_cartpole_diff.xml')

        # alias derivatives and dynamic solver
        self.dynamics_solver = problem.get_scene().get_dynamics_solver()
        # self.fx = jacobian(TestCartpoleDiff.dynamics, argnum=0)
        # self.fu = jacobian(TestCartpoleDiff.dynamics, argnum=1)

    # NOTE: We need autograd for this to work.
    #  PR link: https://github.com/ros/rosdistro/pull/21332
    # def test_diff_autograd(self):
    #     # set the seed so this is deterministic
    #     np.random.seed(42)

    #     # check against 100 state,control pairs
    #     for i in range(100):
    #         x = np.random.uniform(size=(1, 4))[0]
    #         u = np.array([np.random.uniform()])
    #         auto_diff = self.fx(x, u)
    #         solver_diff = self.dynamics_solver.fx(x, u)
    #         nptest.assert_allclose(auto_diff, solver_diff, err_msg="Derivative w.r.t. state test failed")

    #         auto_diff = self.fu(x, u)
    #         solver_diff = self.dynamics_solver.fu(x, u)
    #         nptest.assert_allclose(auto_diff, solver_diff, err_msg="Derivative w.r.t. controls test failed")

    def test_diff_finite_differences(self):
        # set the seed so this is deterministic
        np.random.seed(42)
        eps = 1e-5

        # check against 100 state,control pairs
        for _ in range(100):
            x = np.random.uniform(size=(1, 4))[0]
            u = np.array([np.random.uniform()])
            fx_fd = []
            for i in range(4):
                x_low, x_high = np.copy(x), np.copy(x)
                x_low[i] -= eps/2
                x_high[i] += eps/2
                fx_ = (TestCartpoleDiff.dynamics(x_high, u) - \
                    TestCartpoleDiff.dynamics(x_low, u)) / eps
                fx_fd.append(fx_)

            fx_fd = np.array(fx_fd).T
            fx_solver = self.dynamics_solver.fx(x, u)
            nptest.assert_allclose(fx_fd, fx_solver, err_msg="Derivative w.r.t. state test failed")

            fu_fd = []
            for i in range(1):
                u_low, u_high = np.copy(u), np.copy(u)
                u_low[i] -= eps/2
                u_high[i] += eps/2
                fu_ = (TestCartpoleDiff.dynamics(x, u_high) - \
                    TestCartpoleDiff.dynamics(x, u_low)) / eps
                fu_fd.append(fu_)

            fu_fd = np.array(fu_fd).T
            fu_solver = self.dynamics_solver.fu(x, u)
            nptest.assert_allclose(fu_fd, fu_solver, err_msg="Derivative w.r.t. controls test failed")


if __name__ == '__main__':
    unittest.main()
