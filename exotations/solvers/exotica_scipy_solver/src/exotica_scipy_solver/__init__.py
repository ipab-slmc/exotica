# -*- coding: utf-8 -*-
from __future__ import print_function, division

from scipy.optimize import minimize, Bounds
import numpy as np

class SciPyEndPoseSolver(object):
    def __init__(self, problem=None):
        print("Initialising SciPy Solver")
        self.problem = problem
    
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
        return self.problem.get_inequality()
    
    def neq_constraint_jac(self, x):
        self.problem.update(x)
        return self.problem.get_inequality_jacobian()
    
    def cost_fun(self, x):
        self.problem.update(x)
        return -1. * self.problem.get_scalar_cost()
    
    def cost_jac(self, x):
        self.problem.update(x)
        return -1. * self.problem.get_scalar_jacobian()
    
    def solve(self):
        # Bounds
        bounds = Bounds(self.problem.get_bounds()[:,0], self.problem.get_bounds()[:,1])

        # Extract start state
        x0 = self.problem.start_state.copy()

        self.problem.pre_update()

        # Add constraints
        cons = ()
        if (self.neq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
            cons += ({'type': 'ineq', 'fun': self.neq_constraint_fun, 'jac': self.neq_constraint_jac},)
        
        # if (self.eq_constraint_fun(np.zeros((self.problem.N,))).shape[0] > 0):
        #     cons += ({'type': 'eq', 'fun': self.eq_constraint_fun, 'jac': self.eq_constraint_jac},)

        # print(cons)
        # exit()

        res = minimize(self.cost_fun, 
                    x0,
                    method='COBYLA', # COBYLA
                    jac=self.cost_jac,
                    constraints=cons,
                    options={'disp': True})

        # Solve!
        return res.x

        # Return solution

