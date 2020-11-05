from __future__ import print_function, division
import numpy as np
import pyexotica as exo

NUM_TESTS = 1000


def num_diff_1d(method, x, arg):
    eps = 1e-6
    x_plus = method(x + eps/2.0, arg)
    x_minus = method(x - eps/2.0, arg)
    dx = (x_plus - x_minus) / eps
    return dx


def check_derivative_1d(f, df, arg):
    for _ in range(NUM_TESTS):
        x = np.random.random((1,))
        dx_analytic = df(x, arg)
        dx_numdiff = num_diff_1d(f, x, arg)
        # that's a huge error margin!
        np.testing.assert_allclose(dx_analytic, dx_numdiff, rtol=1e-3)


def test_huber():
    for delta in [0.001, 0.01, 0.1, 1.0, 10.0]:
        check_derivative_1d(exo.Tools.SparseCosts.huber_cost,
                            exo.Tools.SparseCosts.huber_jacobian, delta)
        check_derivative_1d(exo.Tools.SparseCosts.huber_jacobian,
                            exo.Tools.SparseCosts.huber_hessian, delta)


def test_smooth_l1():
    for alpha in [1.0, 10.0]:
        check_derivative_1d(exo.Tools.SparseCosts.smooth_l1_cost,
                            exo.Tools.SparseCosts.smooth_l1_jacobian, alpha)
        check_derivative_1d(exo.Tools.SparseCosts.smooth_l1_jacobian,
                            exo.Tools.SparseCosts.smooth_l1_hessian, alpha)


if __name__ == "__main__":
    test_huber()
    test_smooth_l1()
