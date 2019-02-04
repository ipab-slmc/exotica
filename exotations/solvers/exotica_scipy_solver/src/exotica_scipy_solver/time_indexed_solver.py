# -*- coding: utf-8 -*-
from __future__ import print_function, division

from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, BFGS, SR1
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
    
    def specifyProblem(self, problem):
        self.problem = problem
    
    def eq_constraint_fun(self, x):
        self.problem.update(x)
        return self.problem.get_equality()
    
    def eq_constraint_jac(self, x):
        self.problem.update(x)
        return self.problem.get_equality_jacobian
    
    def neq_constraint_fun(self, x):
        self.problem.update(x)
        # print("NEQ", -1. * self.problem.get_inequality())
        return -1. * self.problem.get_inequality()
    
    def neq_constraint_jac(self, x):
        self.problem.update(x)
        return -1. * self.problem.get_inequality_jacobian()
    
    def cost_fun(self, x):
        for t in xrange(self.problem.T):
            self.problem.update(x[t*self.problem.N:(t+1)*self.problem.N], t)
        # cost = 0.0
        # jac = np.zeros((self.problem.N * self.problem.T,))
        # for t in xrange(self.problem.T):
        #     cost += self.problem.get_scalar_task_cost(t) + self.problem.get_scalar_transition_cost(t)
        return self.problem.get_cost(), self.problem.get_cost_jacobian()
    
    def cost_jac(self, x):
        for t in xrange(self.problem.T):
            self.problem.update(x[t*self.problem.N:(t+1)*self.problem.N], t)
        return self.problem.get_cost_jacobian()
    
    def solve(self):
        # Extract start state
        x0 = self.problem.initial_trajectory

        # Add constraints
        cons = []
        if self.method != "trust-constr":
            if (self.problem.inequality.length_Phi > 0):
                print("Adding NEQ")
                cons.append({'type': 'ineq', 'fun': self.neq_constraint_fun, 'jac': self.neq_constraint_jac})
            
            if (self.problem.equality.length_Phi):
                print("Adding EQ")
                cons.append({'type': 'eq', 'fun': self.eq_constraint_fun, 'jac': self.eq_constraint_jac})
        else:
            if (self.problem.inequality.length_Phi > 0):
                cons.append(NonlinearConstraint(self.neq_constraint_fun, 0., np.inf, jac=self.neq_constraint_jac, hess=SR1()))
            
            if (self.problem.equality.length_Phi):
                cons.append(NonlinearConstraint(self.eq_constraint_fun, 0., 0., jac=self.eq_constraint_jac, hess=SR1()))
            

        # Bounds
        bounds = None
        if self.problem.use_bounds:
            bounds = Bounds(self.problem.get_bounds()[:,0].repeat(self.problem.T), self.problem.get_bounds()[:,1].repeat(self.problem.T))

        s = time()
        res = minimize(self.cost_fun, 
                    x0,
                    # method=self.method,
                    bounds=bounds,
                    jac=True,
                    hess=SR1(),
                    constraints=cons,
                    options={'disp': self.debug, 'initial_tr_radius':1000., 'verbose': 2})
        e = time()
        if self.debug:
            print(e-s)

        traj = np.zeros((self.problem.T, self.problem.N))
        for t in xrange(self.problem.T):
            traj[t,:] = res.x[t*self.problem.N:(t+1)*self.problem.N]
        return traj
