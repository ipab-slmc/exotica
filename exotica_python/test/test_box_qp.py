import numpy as np
import pyexotica as exo
import unittest
from numpy import testing as nptest
from scipy.optimize import minimize

NUM_TESTS = 1000


def check_boxqp_vs_scipy(H, q, b_low, b_high, x_init,
                         threshold_step_acceptance=0.1,
                         max_iterations=100,
                         threshold_gradient_tolerance=1e-5,
                         regularization=1e-12,
                         scipy_method='TNC'):
    check_boxqp_vs_scipy_impl(H, q, b_low, b_high, x_init, threshold_step_acceptance, max_iterations, threshold_gradient_tolerance, regularization, scipy_method, exo.box_qp)
    check_boxqp_vs_scipy_impl(H, q, b_low, b_high, x_init, threshold_step_acceptance, max_iterations, threshold_gradient_tolerance, regularization, scipy_method, exo.box_qp_old)


def check_boxqp_vs_scipy_impl(H, q, b_low, b_high, x_init,
                              threshold_step_acceptance=0.1,
                              max_iterations=100,
                              threshold_gradient_tolerance=1e-5,
                              regularization=0,
                              scipy_method='TNC',
                              box_qp=exo.box_qp):
    sol = box_qp(H, q, b_low, b_high, x_init, threshold_step_acceptance, max_iterations, threshold_gradient_tolerance, regularization)

    def cost(x):
        return .5 * np.matmul(np.matmul(x.T, H), x) + np.matmul(q.T, x)

    # TODO: This is hard-coded for 2D right now!
    sp_sol = minimize(cost, x_init, method=scipy_method, bounds=[
        (b_low[0], b_high[0]),
        (b_low[1], b_high[1]),
    ])

    nptest.assert_allclose(sp_sol.x, sol.x, rtol=1, atol=1e-3, err_msg="BoxQP and SciPy (" + scipy_method + ") differ!")


class TestBoxQP(unittest.TestCase):
    """Tests BoxQP implementation against scipy.""" 
    
    def test_zero_q(self):
        for _ in range(NUM_TESTS):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H)
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.random.uniform(low=-5, high=5, size=(2,))
            q = np.array([0.0, 0.0])

            #check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='TNC')
            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='L-BFGS-B')
    
    def test_zero_h(self):
        for _ in range(NUM_TESTS):
            H = np.array([[0.,0.], [0.,0.]])

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.random.normal(size=(2,1), loc=0, scale=10)

            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='TNC')
            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='L-BFGS-B')

    def test_big_numbers(self):
        for _ in range(NUM_TESTS):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H) * 1e20
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.array([0, 0])

            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='TNC')
            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='L-BFGS-B')

    def test_small_numbers(self):
        for _ in range(NUM_TESTS):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H) * 1e-20
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.array([0, 0])

            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='TNC')
            check_boxqp_vs_scipy(H, q, b_low, b_high, x_init, scipy_method='L-BFGS-B')

if __name__ == '__main__':
    unittest.main()
