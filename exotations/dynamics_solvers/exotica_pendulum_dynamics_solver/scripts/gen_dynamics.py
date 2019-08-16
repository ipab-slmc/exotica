from sympy import *
from pprint import pprint

theta = Symbol('theta')
theta_dot = Symbol('thetadot')
u = Symbol('u')
m_ = Symbol('m_')
g_ = Symbol('g_')
b_ = Symbol('b_')
l_ = Symbol('l_')

theta_ddot = (u - m_ * g_ * l_ * sin(theta) - b_ * theta_dot) / (m_ * l_ * l_)
f = [theta_dot, theta_ddot]
fx = [
    # d/dtheta, d/dthetadot
    [0, 1],
    [diff(theta_ddot, theta), diff(theta_ddot, theta_dot)]
]

fu = [
    0, diff(theta_ddot, u)
]

pprint(fx)
pprint(fu)
