# coding: utf-8
from __future__ import print_function, division

import numpy as np
import pyexotica as exo
import sys

__all__ = ["check_dynamics_solver_derivatives"]

# np.random.seed(42)
np.set_printoptions(4, suppress=True, threshold=sys.maxsize, linewidth=500)

def random_quaternion():
    # Using http://planning.cs.uiuc.edu/node198.html
    uvw = np.random.uniform(size=(3,))
    return np.array([np.sqrt(1.0-uvw[0]) * np.sin(2.0*np.pi*uvw[1]),
                     np.sqrt(1.0-uvw[0]) * np.cos(2.0*np.pi*uvw[1]),
                     np.sqrt(uvw[0])*np.sin(2.0*np.pi*uvw[2]),
                     np.sqrt(uvw[0])*np.cos(2.0*np.pi*uvw[2])])


def random_state(ds):
    x = np.random.random((ds.nx,))

    # Use random quaternion when floating base is represented using SE(3)
    # and scene.get_kinematic_tree().get_model_base_type() == exo.BaseType.Floating:
    if ds.ndx != ds.nq + ds.nv:
        x[3:7] = random_quaternion()
        x[3:7] /= np.linalg.norm(x[3:7])

    return x


def explicit_euler(x, dx, dt, ds=None):
    return x + dt * dx


def semiimplicit_euler(x, dx, dt, ds=None):
    if ds is None:
        raise RuntimeError("ds is None!")
    v = x[ds.nq:].copy()
    a = dx[ds.nq:].copy()

    dx_new = np.concatenate([dt * v + (dt*dt) * a, dt * a])
    return x + dx_new


def check_dynamics_solver_derivatives(name, urdf=None, srdf=None, joint_group=None, additional_args=None, do_test_integrators=True):
    ds = None
    if urdf is not None and srdf is not None and joint_group is not None:
        my_scene_init = exo.Initializers.SceneInitializer()
        my_scene_init[1]['URDF'] = urdf
        my_scene_init[1]['SRDF'] = srdf
        my_scene_init[1]['JointGroup'] = joint_group
        my_scene_init[1]['DynamicsSolver'] = [(name, {'Name': u'MyDynamicsSolver'})]
        if additional_args is not None:
            my_scene_init[1]['DynamicsSolver'][0][1].update(additional_args)
        scene = exo.Setup.create_scene(exo.Initializers.Initializer(my_scene_init))
        ds = scene.get_dynamics_solver()
    else:
        my_ds_init = (name, {'Name': u'MyDynamicsSolver'})
        if additional_args is not None:
            my_ds_init.merge(additional_args)
        ds = exo.Setup.create_dynamics_solver(my_ds_init)

    # Get random state, control
    x = random_state(ds)
    u = np.random.random((ds.nu,))

    # f should return tangent vector type
    np.testing.assert_equal(ds.f(x,u).shape[0], ds.ndx)
    # fx should be (ds.ndx,ds.ndx)
    np.testing.assert_equal(ds.fx(x,u).shape[0], ds.ndx)
    np.testing.assert_equal(ds.fx(x,u).shape[1], ds.ndx)
    # fu should be (ds.ndx,ds.nu)
    np.testing.assert_equal(ds.fu(x,u).shape[0], ds.ndx)
    np.testing.assert_equal(ds.fu(x,u).shape[1], ds.nu)

    # Check integration / simulate
    dx = ds.f(x,u)
    np.testing.assert_array_equal(ds.simulate(x, u, 0.01), ds.integrate(x, dx, 0.01))

    # Checking finite difference derivatives

    ## fu
    fu = ds.fu(x,u)
    fu_fd = ds.fu_fd(x,u)
    # if np.linalg.norm(fu-fu_fd) > 1e-3 or np.any(np.isnan(fu)):
    #     print(fu-fu_fd<1e-3)
    #     print(fu-fu_fd)
    #     print("fu\n",fu)
    #     print("fu_fd\n",fu_fd)
    np.testing.assert_allclose(fu, fu_fd, rtol=1e-5, atol=1e-5)

    ## fx
    fx = ds.fx(x,u)
    fx_fd = ds.fx_fd(x,u)
    if np.linalg.norm(fx-fx_fd) > 1e-3 or np.any(np.isnan(fx)):
        print(fx-fx_fd<1e-3)
        fx_fd[fx_fd<1e-6] = 0.
        print(fx-fx_fd, 2)
        print("fx\n",fx)
        print("fx_fd\n",fx_fd)
    np.testing.assert_allclose(fx, fx_fd, rtol=1e-5, atol=1e-5, err_msg='fx does not match!')

    # Check joint computation
    ds.compute_derivatives(x, u)
    fx_joint = ds.get_fx()
    fu_joint = ds.get_fu()
    np.testing.assert_allclose(fx, fx_joint, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(fu, fu_joint, rtol=1e-5, atol=1e-5)

    # Check different integration schemes
    if ds.nq == ds.nv and do_test_integrators:
        python_integrators = {
            exo.Integrator.RK1: explicit_euler,
            exo.Integrator.SymplecticEuler: semiimplicit_euler
        }
        for integrator in [exo.Integrator.RK1, exo.Integrator.SymplecticEuler]:
            ds.integrator = integrator
            for dt in [0.001, 0.01, 1.0]:
                print("Testing integrator", integrator, "dt=", dt)
                np.testing.assert_allclose(ds.integrate(
                    x, dx, dt), python_integrators[integrator](x, dx, dt, ds))

    # Check state transition function and its derivative for each integration scheme
    for integrator in [exo.Integrator.RK1, exo.Integrator.SymplecticEuler]:
        print("Testing state transition for", integrator, "dt=", ds.dt)
        ds.integrator = integrator
        eps = 1e-5
        Fx_fd = np.zeros((ds.ndx, ds.ndx))
        for i in range(ds.ndx):
            dx = np.zeros((ds.ndx))
            dx[i] = eps / 2.0
            # For finite-diff, we need to use RK1 to compute x_plus, x_minus
            ds.integrator = exo.Integrator.RK1
            x_plus = ds.integrate(x, dx, 1.0)
            x_minus = ds.integrate(x, -dx, 1.0)
            ds.integrator = integrator
            F_plus = ds.F(x_plus, u)
            F_minus = ds.F(x_minus, u)
            Fx_fd[:,i] = ds.state_delta(F_plus, F_minus) / eps

        Fu_fd = np.zeros((ds.ndx, ds.nu))
        for i in range(ds.nu):
            du = np.zeros((ds.nu))
            du[i] = eps / 2.0
            u_plus = u + du
            u_minus = u - du
            F_plus = ds.F(x, u_plus)
            F_minus = ds.F(x, u_minus)
            Fu_fd[:,i] = ds.state_delta(F_plus, F_minus) / eps

        # Run several times to catch the "adding rather than resetting bug"
        ds.compute_derivatives(x, u)
        ds.compute_derivatives(x, u)
        ds.compute_derivatives(x, u)

        # print("Fx_fd\n", Fx_fd)
        # print("ds.get_Fx()\n", ds.get_Fx())
        # print("Fx-diff\n", (Fx_fd-ds.get_Fx()))
        # print(np.abs(Fx_fd-ds.get_Fx()) < 1e-5)
        # print("Fu_fd\n", Fu_fd)
        # print("ds.get_Fu()\n", ds.get_Fu())
        # print("Fu-diff\n", (Fu_fd-ds.get_Fu()))
        # print(np.abs(Fu_fd-ds.get_Fu()) < 1e-5)

        np.testing.assert_allclose(ds.get_Fx(), Fx_fd, rtol=1e-5, atol=1e-5)
        np.testing.assert_allclose(ds.get_Fu(), Fu_fd, rtol=1e-5, atol=1e-5)

    # Check state delta and its derivatives
    if ds.nq != ds.nv:
        #print("Non-Euclidean space: Check StateDelta")
        x1 = random_state(ds)
        x2 = random_state(ds)

        # Check dStateDelta
        Jds = ds.state_delta_derivative(x1, x2, exo.ArgumentPosition.ARG0)
        Jdiff = np.zeros((ds.ndx, ds.ndx))
        eps = 1e-5
        integrator = ds.integrator
        for i in range(ds.ndx):
            dx = np.zeros((ds.ndx))
            dx[i] = eps / 2.0
            ds.integrator = exo.Integrator.RK1
            x1_plus = ds.integrate(x1, dx, 1.0)
            x1_minus = ds.integrate(x1, -dx, 1.0)
            ds.integrator = integrator
            delta_plus = ds.state_delta(x1_plus, x2)
            delta_minus = ds.state_delta(x1_minus, x2)
            Jdiff[:,i] = (delta_plus - delta_minus) / eps
        np.testing.assert_allclose(Jds, Jdiff, rtol=1e-5, atol=1e-5)

        # Check ddStateDelta
        # TODO: Verify
        x1 = random_state(ds)
        x2 = random_state(ds)
        Hds = ds.state_delta_second_derivative(x1, x2, exo.ArgumentPosition.ARG0)
        Hdiff = np.zeros((ds.ndx, ds.ndx, ds.ndx))
        eps = 1e-5
        integrator = ds.integrator
        for i in range(ds.ndx):
            dx = np.zeros((ds.ndx))
            dx[i] = eps / 2.0
            ds.integrator = exo.Integrator.RK1
            x1_plus = ds.integrate(x1, dx, 1.0)
            x1_minus = ds.integrate(x1, -dx, 1.0)
            ds.integrator = integrator
            Jdiff_plus = ds.state_delta_derivative(x1_plus, x2, exo.ArgumentPosition.ARG0)
            Jdiff_minus = ds.state_delta_derivative(x1_minus, x2, exo.ArgumentPosition.ARG0)
            Hdiff[:,:,i] = (Jdiff_plus - Jdiff_minus) / eps
        np.testing.assert_allclose(Hds, Hdiff, rtol=1e-5, atol=1e-5)
