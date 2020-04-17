from sympy import *
# from pprint import pprint
from sympy.matrices import *
import numpy as np

x_ = Symbol('x_')
y_ = Symbol('y_')
z_ = Symbol('z_')
phi = Symbol('phi')
theta = Symbol('theta')
psi = Symbol('psi')
x_dot = Symbol('x_dot')
y_dot = Symbol('y_dot')
z_dot = Symbol('z_dot')
phi_dot = Symbol('phi_dot')
theta_dot = Symbol('theta_dot')
psi_dot = Symbol('psi_dot')
k_f_ = Symbol('k_f_')
k_m_ = Symbol('k_m_')
u0 = Symbol('u(0)')
u1 = Symbol('u(1)')
u2 = Symbol('u(2)')
u3 = Symbol('u(3)')
g_ = Symbol('g_')
mass_ = Symbol('mass_')
L_ = Symbol('L_')
# b_ = Symbol('b_')

F_1 = k_f_ * u0
F_2 = k_f_ * u1
F_3 = k_f_ * u2
F_4 = k_f_ * u3

M_1 = k_m_ * u0
M_2 = k_m_ * u1
M_3 = k_m_ * u2
M_4 = k_m_ * u3

sin_phi = sin(phi)
cos_phi = cos(phi)
tan_phi = tan(phi)
sin_theta = sin(theta)
cos_theta = cos(theta)
tan_theta = tan(theta)
sin_psi = sin(psi)
cos_psi = cos(psi)
tan_psi = tan(psi)

Rx = Matrix([
    [1, 0, 0],
    [0, cos_phi, -sin_phi],
    [0, sin_phi, cos_phi]
])
Ry = Matrix([
    [cos_theta, 0, sin_theta],
    [0, 1, 0],
    [-sin_theta, 0, cos_theta]
])
Rz = Matrix([
    [cos_psi, -sin_psi, 0],
    [sin_psi, cos_psi, 0],
    [0, 0, 1]
])

R = Rz * Rx * Ry

Gtot = Matrix([0, 0, - g_])
Ftot = Matrix([0, 0, F_1 + F_2 + F_3 + F_4])

pos_ddot = Gtot + R * Ftot / mass_
# pprint(R)

# phi, theta, psi dynamics
tau = Matrix([
    L_ * (F_1 - F_2), L_ * (F_1 - F_3),
    # b_ * (M_1 - M_2 + M_3 - M_4)
    (M_1 - M_2 + M_3 - M_4)
])
omega = Matrix([phi_dot, theta_dot, psi_dot])

radius = L_ / 2.0
Ix = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_
Iy = 2 * mass_ * (radius * radius) / 5.0 + 2 * radius * radius * mass_
Iz = 2 * mass_ * (radius * radius) / 5.0 + 4 * radius * radius * mass_

# Eigen::Matrix3d I, Iinv;
I = Matrix([[Ix, 0, 0], [0, Iy, 0], [0, 0, Iz]])
Iinv = Matrix([[1 / Ix, 0, 0], [0, 1 / Iy, 0], [0, 0, 1 / Iz]])

omega_dot = Iinv * (tau - omega.cross(I * omega))

    # Eigen::VectorXd state_dot(12);
state_dot = Matrix([
    x_dot, y_dot, z_dot,
    phi_dot, theta_dot, psi_dot,
    pos_ddot[0], pos_ddot[1], pos_ddot[2],
    omega_dot[0], omega_dot[1], omega_dot[2]
])

statevars = [
    x_, y_, z_, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot
]
fx = []

for var in statevars:
    fx.append(diff(state_dot, var).T)

fx = Matrix(np.array(fx).T.tolist())

print('fx')
print('=' * 40)
for i in range(12):
    print(list(fx.row(i)))


print('')
print('fu')
print('=' * 40)

fu = []

for var in [u0, u1, u2, u3]:
    fu.append(diff(state_dot, var).T)

fu = Matrix(np.array(fu).T.tolist())
# print(fu)
for i in range(12):
    print(list(fu.row(i)))
