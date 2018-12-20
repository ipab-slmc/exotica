#!/usr/bin/env python
import pyexotica as exo
from pyexotica.publish_trajectory import *
import signal
import time

if __name__ == '__main__':
    exo.Setup.init_ros("shape_example")
    solver = exo.Setup.load_solver('{exotica_examples}/resources/configs/ompl_solver_demo.xml')

    # colours are given in tuples (red, green, blue, alpha) in range 0.0...1.0
    alpha = 1.0
    black = exo.msgColorRGBA(0, 0, 0, alpha)
    white = exo.msgColorRGBA(1, 1, 1, alpha)
    green = exo.msgColorRGBA(0, 1, 0, alpha)
    orange = exo.msgColorRGBA(1, 0.5, 0, alpha)

    solver.get_problem().get_scene().add_object_to_environment(
                name="plane", colour=green, shape=exo.Box(2, 2, 0.01))

    solver.get_problem().get_scene().add_object_to_environment(
                name="sphere1", colour=white,
                shape=exo.Sphere(0.4), transform=exo.KDLFrame([0, 0, 0.4]))

    solver.get_problem().get_scene().add_object_to_environment(
                name="sphere2", colour=white,
                shape=exo.Sphere(0.25), transform=exo.KDLFrame([0, 0, 0.95]))

    solver.get_problem().get_scene().add_object_to_environment(
                name="sphere3", colour=white,
                shape=exo.Sphere(0.15), transform=exo.KDLFrame([0, 0, 1.3]))

    dots = ( [0, 0.375, 0.55], [0, 0.28, 0.68], [0, 0.203, 0.812], [0, 0.25, 0.95], [0, 0.21, 1.07])
    for idot, coord in enumerate(dots):
        solver.get_problem().get_scene().add_object_to_environment(
                    name="dotb"+str(idot), colour=black,
                    shape=exo.Sphere(0.01), transform=exo.KDLFrame(coord))

    # transformation given as position (x,y,z) + quaternion (w,x,y,z)
    solver.get_problem().get_scene().add_object_to_environment(
                name="cone1", colour=orange,
                shape=exo.Cone(0.01, 0.1), transform=exo.KDLFrame([0, 0.15, 1.3, 1, 0, 0, 1]))

    solver.get_problem().get_scene().add_object_to_environment(
                name="cylinder1", colour=black,
                shape=exo.Cylinder(0.15, 0.01), transform=exo.KDLFrame([0, 0, 1.43]))

    solver.get_problem().get_scene().add_object_to_environment(
                name="cylinder2", colour=black,
                shape=exo.Cylinder(0.08, 0.1), transform=exo.KDLFrame([0, 0, 1.48]))

    dots = ([0.036, 0.132, 1.36], [-0.036, 0.132, 1.36])
    for idot, coord in enumerate(dots):
        solver.get_problem().get_scene().add_object_to_environment(
                    name="dote"+str(idot), colour=black,
                    shape=exo.Sphere(0.015), transform=exo.KDLFrame(coord))

    dots = ([0, 0.142, 1.252], [0.022, 0.143, 1.256], [-0.022, 0.143, 1.256], [0.043, 0.141, 1.263], [-0.043, 0.141, 1.263])
    for idot, coord in enumerate(dots):
        solver.get_problem().get_scene().add_object_to_environment(
                    name="dotm"+str(idot), colour=black,
                    shape=exo.Sphere(0.01), transform=exo.KDLFrame(coord))

    # publish shapes until SIGINT (e.g. Ctrl+C) is received
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        try:
            solver.get_problem().get_scene().get_kinematic_tree().publish_frames()
            time.sleep(0.1)
        except KeyboardInterrupt:
            break