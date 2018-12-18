#!/usr/bin/env python
from __future__ import print_function
import rospy
import pyexotica as exo
import numpy as np
import signal
from pyexotica.publish_trajectory import publish_pose, sig_int_handler
from time import sleep
import task_map_py
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray as FloatArray

DT = 1.0/100.0  # 100 HZ
DAMP = 0.005

SMOOTHING_TASK_MAPS = ['JointVel', 'JointAcc', 'JointJerk']

XLIM = [0.0, 0.75]
YLIM = [-0.5, 0.5]
ZLIM = [0, 1.0]

LIMITS = np.array([XLIM,
                   YLIM,
                   ZLIM])


class Example(object):

    def __init__(self):

        # Set init variables
        self.q = np.array([0.0] * 7)
        self.joy = None
        self.eff = np.array([0.6, -0.1, 0.5, 0, 0, 0])

        # Setup exotica
        self.solver = exo.Setup.load_solver(
            '{exotica_examples}/resources/configs/ik_with_jnt_smoothing.xml')
        self.problem = self.solver.get_problem()
        self.task_maps = self.problem.get_task_maps()

        # Setup ros subscriber
        self.sub = {}
        self.sub['joy'] = rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, msg):
        self.joy = msg

    def update(self, event):
        if self.joy is None:
            return

        # Compute eff goal
        dx = self.joy.axes[4]
        dy = self.joy.axes[0]
        dz = self.joy.axes[1]
        eff = self.eff + DAMP * np.array([dx, dy, dz, 0, 0, 0])

        # Check limits
        for i in xrange(3):
            if not LIMITS[i, 0] <= eff[i] <= LIMITS[i, 1]:
                eff[i] = self.eff[i]

        # Set eff goal
        self.problem.get_scene().attach_object_local('Goal', '', exo.KDLFrame(eff))

        # Setup problem
        if self.joy.buttons[0]:
            # Button [0] pressed -> turn smoothing off -> report
            Rho_Smoothing = 0
            print("Smoothing OFF")
        else:
            # Button [0] released -> turn smoothing on -> report
            Rho_Smoothing = 1
            print("Smoothing ON")

        # Set start state, rho, and previous joint states
        self.problem.start_state = self.q
        for name in SMOOTHING_TASK_MAPS:
            self.problem.set_rho(name, Rho_Smoothing)
            self.task_maps[name].set_previous_joint_state(self.q)

        # Solve and publish
        q = self.solver.solve()[0]
        publish_pose(q, self.problem)

        # Set new as old
        self.q = q
        self.eff = eff


if __name__ == '__main__':
    rospy.init_node('example_ik_with_jnt_smoothing_node')
    exo.Setup.init_ros()
    signal.signal(signal.SIGINT, sig_int_handler)
    rospy.Timer(rospy.Duration(DT), Example().update)
    rospy.spin()
