# -*- coding: utf-8 -*-
from __future__ import print_function, division

from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, BFGS, SR1
import numpy as np
from time import time


class SciPyEndPoseSolver(object):
    '''
    Uses SciPy to solve a constrained EndPoseProblem. Options for SciPy minimize
    can be found here:
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
    '''
    def __init__(self, problem=None, method=None, debug=False):
        print("Initialising SciPy Solver")
        self.problem = problem
        self.debug = debug
        self.method = method
        self.max_iterations = 100
    
    def specifyProblem(self, problem):
        self.problem = problem
    
    def eq_constraint_fun(self, x):
        self.problem.update(x)
        return self.problem.get_equality()
    
    def eq_constraint_jac(self, x):
        self.problem.update(x)
        return self.problem.get_equality_jacobian()
    
    def neq_constraint_fun(self, x):
        self.problem.update(x)
        # print("NEQ", -1. * self.problem.get_inequality())
        return -1. * self.problem.get_inequality()
    
    def neq_constraint_jac(self, x):
        self.problem.update(x)
        return -1. * self.problem.get_inequality_jacobian()
    
    def cost_fun(self, x):
        self.problem.update(x)
        return self.problem.get_scalar_cost(), self.problem.get_scalar_jacobian()
    
    def cost_jac(self, x):
        self.problem.update(x)
        return self.problem.get_scalar_jacobian()
    
    def solve(self):
        # Extract start state
        x0 = self.problem.start_state.copy()

        self.problem.pre_update()

        # Add constraints
        cons = []
        if self.method != "trust-constr":
            if (self.neq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
                cons.append({'type': 'ineq', 'fun': self.neq_constraint_fun, 'jac': self.neq_constraint_jac})
            
            if (self.eq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
                cons.append({'type': 'eq', 'fun': self.eq_constraint_fun, 'jac': self.eq_constraint_jac})
        else:
            if (self.neq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
                cons.append(NonlinearConstraint(self.neq_constraint_fun, 0., np.inf, jac=self.neq_constraint_jac, hess=SR1()))
            
            if (self.eq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
                cons.append(NonlinearConstraint(self.eq_constraint_fun, 0., 0., jac=self.eq_constraint_jac, hess=SR1()))


        # Bounds
        bounds = None
        if self.problem.use_bounds:
            bounds = Bounds(self.problem.get_bounds()[:,0], self.problem.get_bounds()[:,1])

        s = time()
        res = minimize(self.cost_fun,
                    x0,
                    method=self.method,
                    bounds=bounds,
                    jac=True,
                    hess=SR1(),
                    constraints=cons,
                    options={'disp': self.debug, 'initial_tr_radius':1000., 'maxiter': self.max_iterations})
        e = time()
        if self.debug:
            print(e-s, res.x)

        return [res.x]
