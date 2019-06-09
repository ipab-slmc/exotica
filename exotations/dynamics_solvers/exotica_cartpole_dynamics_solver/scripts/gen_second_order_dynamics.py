from sympy import *
from pprint import pprint

theta = Symbol('theta')
tdot = Symbol('tdot')
xdot = Symbol('xdot')
u = Symbol('u')
m_p_ = Symbol('m_p_')
m_c_ = Symbol('m_c_')
g_ = Symbol('g_')
l_ = Symbol('l_')

xddot = (u + m_p_ * sin(theta) * (l_ * (tdot * tdot) + g_ * cos(theta))) / (m_c_ + m_p_ * (sin(theta) * sin(theta)))
tddot = - (l_ * m_p_ * cos(theta) * sin(theta) * (tdot * tdot) + u * cos(theta) + (m_c_ + m_p_) * g_ * \
    sin(theta))/ (l_ * m_c_ + l_ * m_p_ * (sin(theta)*sin(theta)))

f = [
    xdot, tdot, xddot, tddot
]

fx = [
    # d/dx, d/dtheta, d/dxdot, d/dtdot
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, diff(xddot, theta), 0, diff(xddot, tdot)],
    [0, diff(tddot, theta), 0, diff(tddot, tdot)]
]

fu = [
    0, 0, diff(xddot, u), diff(tddot, u)
]

fuu = [
    0, 0, diff(diff(xddot, u), u), diff(diff(tddot, u), u)
]

fxx = [
    [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ], # fx_x
    [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, diff(diff(xddot, theta), theta), 0, diff(diff(xddot, tdot), theta)],
        [0, diff(diff(tddot, theta), theta), 0, diff(diff(tddot, tdot), theta)]
    ], # fx_theta
    [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ], # fx_xdot
    [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, diff(diff(xddot, theta), tdot), 0, diff(diff(xddot, tdot), tdot)],
        [0, diff(diff(tddot, theta), tdot), 0, diff(diff(tddot, tdot), tdot)]
    ], # fx_tdot
]


fu = [
    0, 0, diff(xddot, u), diff(tddot, u)
]


# fu = 0, 0, diff(xddot, u), diff(tddot, u)
fux = [
    # d/dx, d/dtheta, d/dxdot, d/dtdot
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, diff(diff(xddot, u), theta), 0, diff(diff(xddot, u), tdot)],
    [0, diff(diff(tddot, u), theta), 0, diff(diff(tddot, u), tdot)]
]

fxu = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, diff(diff(xddot, theta), u), 0, diff(diff(xddot, tdot), u)],
    [0, diff(diff(tddot, theta), u), 0, diff(diff(tddot, tdot), u)]
]

pprint(fxx)
pprint(fux)
pprint(fuu)
