# coding: utf-8
import roslib
import unittest
import numpy as np
import pyexotica as exo
from pyexotica.testing import check_dynamics_solver_derivatives

PKG = 'exotica_examples'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.

# Load a problem
config = '{exotica_examples}/resources/configs/dynamic_time_indexed/01_ilqr_cartpole.xml'
# config = '{exotica_examples}/resources/configs/dynamic_time_indexed/02_lwr_task_maps.xml'
# config = '{exotica_examples}/resources/configs/dynamic_time_indexed/03_ilqr_valkyrie.xml'

_, problem_init = exo.Initializers.load_xml_full(config)
problem = exo.Setup.create_problem(problem_init)
ds = problem.get_scene().get_dynamics_solver()

print(problem, ds)
print(ds.nq, ds.nv, ds.nx, ds.ndx)

t = problem.T - 1  # final cost
nq = ds.nq

x_T_minus_1 = problem.X[:,-2].copy()
u_T_minus_1 = problem.U[:,-1].copy()

# test state cost jacobian
problem.update(x_T_minus_1, u_T_minus_1, t - 1)
J_solver = problem.get_state_cost_jacobian(t).copy()
J_numdiff = np.zeros_like(J_solver)
assert J_numdiff.shape[0] == ds.ndx

eps = 1e-6
for i in range(ds.ndx):
    x_plus = x_T_minus_1.copy()
    x_plus[i] += eps / 2.0
    x_minus = x_T_minus_1.copy()
    x_minus[i] -= eps / 2.0

    problem.update(x_plus, u_T_minus_1, t - 1)
    cost_high = problem.get_state_cost(t)

    problem.update(x_minus, u_T_minus_1, t - 1)
    cost_low = problem.get_state_cost(t)

    J_numdiff[i] = (cost_high - cost_low) / eps

np.set_printoptions(6, suppress=True)
print(J_solver)
print(J_numdiff)
np.testing.assert_allclose(J_solver, J_numdiff, rtol=1e-5, atol=1e-5, err_msg='StateCostJacobian does not match!')

# TODO: test state cost hessian

# TODO: test control cost jacobian

# TODO: test control cost hessian

# TODO: test state control cost hessian
