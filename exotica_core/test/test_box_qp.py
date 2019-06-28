import numpy as np
import pyexotica as exo
import unittest
from numpy import testing as nptest
from scipy.optimize import minimize

class TestBoxQP(unittest.TestCase):
    """Tests BoxQP implementation against scipy.""" 
    
    def test_zero_q(self):
        np.random.seed(100)

        # check against 100 state,control pairs
        for i in range(100):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H)
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.array([0.0, 0.0])

            sol = exo.box_qp(H, q, b_low, b_high, x_init, 0.1, 100, 1e-5, 1e-5)
            
            def cost(x):
                return .5 * np.matmul(np.matmul(x.T, H), x) + np.matmul(q.T, x)

            sp_sol = minimize(cost, x_init, method='TNC', bounds=[
                (b_low[0], b_high[0]),
                (b_low[1], b_high[1]),
            ])

            nptest.assert_allclose(sp_sol.x, sol.x.T[0], rtol=1, atol=1e-4, err_msg="BoxQP and scipy differ!")
    
    def test_zero_h(self):
        np.random.seed(100)

        # check against 100 state,control pairs
        for i in range(10):
            H = np.array([[0.,0.], [0.,0.]])

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.random.normal(size=(2,1), loc=0, scale=10)

            sol = exo.box_qp(H, q, b_low, b_high, x_init, 0.1, 100, 1e-5, 1e-5)
            
            def cost(x):
                return .5 * np.matmul(np.matmul(x.T, H), x) + np.matmul(q.T, x)

            sp_sol = minimize(cost, x_init, method='TNC', bounds=[
                (b_low[0], b_high[0]),
                (b_low[1], b_high[1]),
            ])

            nptest.assert_allclose(sp_sol.x, sol.x.T[0], rtol=1, atol=1e-4, err_msg="BoxQP and scipy differ!")

    def test_big_numbers(self):
        np.random.seed(100)

        # check against 100 state,control pairs
        for i in range(100):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H) * 1e20
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.array([0, 0])

            sol = exo.box_qp(H, q, b_low, b_high, x_init, 0.1, 100, 1e-5, 1e-5)
            
            def cost(x):
                return .5 * np.matmul(np.matmul(x.T, H), x) + np.matmul(q.T, x)

            sp_sol = minimize(cost, x_init, method='TNC', bounds=[
                (b_low[0], b_high[0]),
                (b_low[1], b_high[1]),
            ])

            nptest.assert_allclose(sp_sol.x, sol.x.T[0], rtol=1, atol=1e-4, err_msg="BoxQP and scipy differ!")

    def test_small_numbers(self):
        np.random.seed(100)

        # check against 100 state,control pairs
        for i in range(10):
            H = np.random.normal(
                size=(2, 2), loc=0, scale=10
            )
            H = np.abs(H) * 1e-20
            H[0, 1] = H[1,0] = 0

            b_low = np.array([-5., -5.])
            b_high = np.array([5., 5.])
            x_init = np.array([-3., 2.])
            q = np.array([0, 0])

            sol = exo.box_qp(H, q, b_low, b_high, x_init, 0.1, 100, 1e-5, 1e-5)
            
            def cost(x):
                return .5 * np.matmul(np.matmul(x.T, H), x) + np.matmul(q.T, x)

            sp_sol = minimize(cost, x_init, method='TNC', bounds=[
                (b_low[0], b_high[0]),
                (b_low[1], b_high[1]),
            ])

            nptest.assert_allclose(sp_sol.x, sol.x.T[0], rtol=1, atol=1e-4, err_msg="BoxQP and scipy differ!")

if __name__ == '__main__':
    unittest.main()
