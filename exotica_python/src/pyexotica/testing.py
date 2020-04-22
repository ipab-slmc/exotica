# coding: utf-8
from __future__ import print_function, division

import numpy as np
import pyexotica as exo

__all__ = ["check_dynamics_solver_derivatives"]

# np.random.seed(42)

def random_quaternion():
    # Using http://planning.cs.uiuc.edu/node198.html
    uvw = np.random.uniform(size=(3,))
    return np.array([np.sqrt(1.0-uvw[0]) * np.sin(2.0*np.pi*uvw[1]),
                     np.sqrt(1.0-uvw[0]) * np.cos(2.0*np.pi*uvw[1]),
                     np.sqrt(uvw[0])*np.sin(2.0*np.pi*uvw[2]),
                     np.sqrt(uvw[0])*np.cos(2.0*np.pi*uvw[2])])

def check_dynamics_solver_derivatives(name, urdf=None, srdf=None, joint_group=None):
    ds = None
    if urdf is not None and srdf is not None and joint_group is not None:
        my_scene_init = exo.Initializers.SceneInitializer()
        my_scene_init[1]['URDF'] = urdf
        my_scene_init[1]['SRDF'] = srdf
        my_scene_init[1]['JointGroup'] = joint_group
        my_scene_init[1]['DynamicsSolver'] = [(name, {'Name': u'MyDynamicsSolver'})]
        scene = exo.Setup.create_scene(exo.Initializers.Initializer(my_scene_init))
        ds = scene.get_dynamics_solver()
    else:
        ds = exo.Setup.create_dynamics_solver((name, {'Name': u'MyDynamicsSolver'}))

    # Check dimensions
    x = np.random.random((ds.nx,))

    # Use random quaternion when floating base is represented using SE(3)
    if ds.ndx != ds.nq + ds.nv and scene.get_kinematic_tree().get_model_base_type() == exo.BaseType.Floating:
        x[3:7] = random_quaternion()

    u = np.random.random((ds.nu,))

    # f should return tangent vector type
    assert ds.f(x,u).shape[0] == ds.ndx
    # fx should be (ds.ndx,ds.ndx)
    assert ds.fx(x,u).shape[0] == ds.ndx and ds.fx(x,u).shape[1] == ds.ndx
    # fu should be (ds.ndx,ds.nu)
    assert ds.fu(x,u).shape[0] == ds.ndx and ds.fu(x,u).shape[1] == ds.nu

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
    # if np.linalg.norm(fx-fx_fd) > 1e-3 or np.any(np.isnan(fx)):
    #     print(fx-fx_fd<1e-3)
    #     print("fx\n",fx)
    #     print("fx_fd\n",fx_fd)
    np.testing.assert_allclose(fx, fx_fd, rtol=1e-5, atol=1e-5)

    # Check joint computation
    ds.compute_derivatives(x, u)
    fx_joint = ds.get_fx()
    fu_joint = ds.get_fu()
    np.testing.assert_allclose(fx, fx_joint, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(fu, fu_joint, rtol=1e-5, atol=1e-5)
