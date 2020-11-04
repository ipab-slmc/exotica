# -*- coding: utf-8 -*-
from __future__ import print_function, division

from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, SR1 # BFGS
import numpy as np
from time import time


class SciPyTimeIndexedSolver(object):
    '''
    Uses SciPy to solve a constrained TimeIndexedProblem. Options for SciPy minimize
    can be found here:
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
    '''

    def __init__(self, problem=None, method=None, debug=False):
        print("Initialising SciPy Solver")
        self.problem = problem
        self.debug = debug
        self.method = method
        self.hessian_update_strategy = SR1()
        self.max_iterations = 500

    def specifyProblem(self, problem):
        self.problem = problem

    def eq_constraint_fun(self, x):
        self.problem.update(x)
        # print("EQ", self.problem.get_equality().shape)
        return self.problem.get_equality()

    def eq_constraint_jac(self, x):
        self.problem.update(x)
        # print("EQ-Jac", self.problem.get_equality_jacobian().shape)
        if self.method == "SLSQP":  # SLSQP does not support sparse Jacobians/Hessians
            return self.problem.get_equality_jacobian().todense()
        else:
            return self.problem.get_equality_jacobian()

    def neq_constraint_fun(self, x):
        self.problem.update(x)
        # print("NEQ", self.problem.get_inequality().shape)
        return -1. * self.problem.get_inequality()

    def neq_constraint_jac(self, x):
        self.problem.update(x)
        # print("NEQ-Jac", self.problem.get_inequality_jacobian().shape)
        if self.method == "SLSQP":  # SLSQP does not support sparse Jacobians/Hessians
            return -1. * self.problem.get_inequality_jacobian().todense()
        else:
            return -1. * self.problem.get_inequality_jacobian()

    def cost_fun(self, x):
        self.problem.update(x)
        return self.problem.get_cost(), self.problem.get_cost_jacobian()

    def solve(self):
        # Extract start state
        x0 = np.asarray(self.problem.initial_trajectory)[1:, :].flatten()
        x0 += np.random.normal(0., 1.e-3, x0.shape[0]) # for SLSQP we do require some initial noise to avoid singular matrices

        # Add constraints
        cons = []
        if self.method != "trust-constr":
            if (self.problem.inequality.length_Phi > 0):
                cons.append({'type': 'ineq', 'fun': self.neq_constraint_fun, 'jac': self.neq_constraint_jac})
            
            if (self.problem.equality.length_Phi):
                cons.append({'type': 'eq', 'fun': self.eq_constraint_fun, 'jac': self.eq_constraint_jac})
        else:
            if (self.problem.inequality.length_Phi > 0):
                cons.append(NonlinearConstraint(self.neq_constraint_fun, 0., np.inf, jac=self.neq_constraint_jac, hess=self.hessian_update_strategy))

            if (self.problem.equality.length_Phi):
                cons.append(NonlinearConstraint(self.eq_constraint_fun, 0., 0., jac=self.eq_constraint_jac, hess=self.hessian_update_strategy))

        # Bounds
        bounds = None
        if self.problem.use_bounds:
            bounds = Bounds(self.problem.get_bounds()[:,0].repeat(self.problem.T - 1), self.problem.get_bounds()[:,1].repeat(self.problem.T - 1))

        s = time()
        res = minimize(self.cost_fun,
                       x0,
                       method=self.method,
                       bounds=bounds,
                       jac=True,
                       hess=self.hessian_update_strategy,
                       constraints=cons,
                       options={
                           'disp': self.debug,
                           'initial_tr_radius': 1000.,
                           'verbose': 2,
                           'maxiter': self.max_iterations
                       })
        e = time()
        if self.debug:
            print(e-s)

        traj = np.zeros((self.problem.T, self.problem.N))
        traj[0, :] = self.problem.start_state
        for t in xrange(0, self.problem.T - 1):
            traj[t + 1, :] = res.x[t*self.problem.N:(t+1)*self.problem.N]
        return traj
