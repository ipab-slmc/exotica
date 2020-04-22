# coding: utf-8
import roslib
import unittest
import numpy as np
import pyexotica as exo
from pyexotica.testing import random_state

PKG = 'exotica_examples'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.

# Load a problem
config = '{exotica_examples}/resources/configs/dynamic_time_indexed/01_ilqr_cartpole.xml'
# config = '{exotica_examples}/resources/configs/dynamic_time_indexed/02_lwr_task_maps.xml'
# config = '{exotica_examples}/resources/configs/dynamic_time_indexed/03_ilqr_valkyrie.xml'

_, problem_init = exo.Initializers.load_xml_full(config)
problem = exo.Setup.create_problem(problem_init)
scene = problem.get_scene()
ds = problem.get_scene().get_dynamics_solver()

print(problem, ds)
print(ds.nq, ds.nv, ds.nx, ds.ndx)


def test_state_cost_terminal_state():
    x = random_state(scene)

    problem.update_terminal_state(x)
    J_solver = problem.get_state_cost_jacobian(-1).copy()
    J_numdiff = np.zeros_like(J_solver)
    assert J_numdiff.shape[0] == ds.ndx

    eps = 1e-6
    for i in range(ds.ndx):
        dx = np.zeros((ds.ndx,))
        dx[i] = eps / 2.0

        x_plus = ds.integrate(x, dx, 1.0)
        x_minus = ds.integrate(x, dx, -1.0)

        problem.update_terminal_state(x_plus)
        cost_high = problem.get_state_cost(-1)

        problem.update_terminal_state(x_minus)
        cost_low = problem.get_state_cost(-1)

        J_numdiff[i] = (cost_high - cost_low) / eps

    # np.set_printoptions(6, suppress=True)
    # print(J_solver)
    # print(J_numdiff)
    np.testing.assert_allclose(J_solver, J_numdiff, rtol=1e-5, atol=1e-5,
                               err_msg='StateCostJacobian at terminal state does not match!')


def test_state_cost_jacobian_at_t(t):
    x = random_state(scene)
    u = np.random.random((ds.nu,))

    problem.update(x, u, t)
    J_solver = problem.get_state_cost_jacobian(t).copy()
    J_numdiff = np.zeros_like(J_solver)
    assert J_numdiff.shape[0] == ds.ndx

    eps = 1e-6
    for i in range(ds.ndx):
        dx = np.zeros((ds.ndx,))
        dx[i] = eps / 2.0

        x_plus = ds.integrate(x, dx, 1.0)
        x_minus = ds.integrate(x, dx, -1.0)

        problem.update(x_plus, u, t)
        cost_high = problem.get_state_cost(t)

        problem.update(x_minus, u, t)
        cost_low = problem.get_state_cost(t)

        J_numdiff[i] = (cost_high - cost_low) / eps
    np.testing.assert_allclose(J_solver, J_numdiff, rtol=1e-5,
                               atol=1e-5, err_msg='StateCostJacobian does not match!')


def test_control_cost_jacobian_at_t(t):
    x = random_state(scene)
    u = np.random.random((ds.nu,))

    problem.update(x, u, t)
    J_solver = problem.get_control_cost_jacobian(t).copy()
    J_numdiff = np.zeros_like(J_solver)
    assert J_numdiff.shape[0] == ds.nu

    eps = 1e-6
    for i in range(ds.nu):
        u_plus = u.copy()
        u_plus[i] += eps / 2.0
        u_minus = u.copy()
        u_minus[i] -= eps / 2.0

        problem.update(x, u_plus, t)
        cost_high = problem.get_control_cost(t)

        problem.update(x, u_minus, t)
        cost_low = problem.get_control_cost(t)

        J_numdiff[i] = (cost_high - cost_low) / eps
    np.testing.assert_allclose(J_solver, J_numdiff, rtol=1e-5,
                               atol=1e-5, err_msg='ControlCostJacobian does not match!')

###############################################################################


# test state cost jacobian at final state
test_state_cost_terminal_state()

# test state cost jacobian
for t in range(problem.T - 1):
    test_state_cost_jacobian_at_t(t)

# TODO: test state cost hessian

# test control cost jacobian
for t in range(problem.T - 1):
    test_control_cost_jacobian_at_t(t)

# TODO: test control cost hessian

# TODO: test state control cost hessian
