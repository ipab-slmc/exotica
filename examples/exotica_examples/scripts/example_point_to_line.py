#!/usr/bin/env python
import rospy
import numpy as np
import pyexotica as exo
import signal
from pyexotica.publish_trajectory import publish_pose, sig_int_handler
import exotica_core_task_maps_py
from sensor_msgs.msg import Joy, JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA as Color

green = Color()
green.r = 0.0
green.g = 1.0
green.b = 0.0
green.a = 1.0

DT = 1.0/100.0  # 100 HZ
DAMP = 0.005

INIT_POSITION = np.array([0.75, 0, 0.5])  # initial position of the end point


def MarkerMsg(p):
    marker = Marker()
    marker.type = marker.SPHERE
    marker.id = 0
    marker.action = marker.ADD
    marker.header.frame_id = 'exotica/world_frame'
    for i, d in enumerate(['x', 'y', 'z']):
        setattr(marker.pose.position, d, p[i])
        setattr(marker.scale, d, 0.15)
    marker.pose.orientation.w = 1.0
    marker.color = green
    return marker


class Example(object):

    def __init__(self):

        self.solver = exo.Setup.load_solver(
            '{exotica_examples}/resources/configs/point_to_line.xml')
        self.problem = self.solver.get_problem()
        self.q = np.array([0.0]*7)

        self.p = INIT_POSITION

        self.pub = {}
        self.pub['marker'] = rospy.Publisher(
            'teleop_point', Marker, queue_size=1)
        self.pub['joint_state'] = rospy.Publisher(
            'joint_states', JointState, queue_size=1)

        self.sub = {}
        self.sub['joy'] = rospy.Subscriber('joy', Joy, self.callback)
        self.joy = None

    def callback(self, msg):
        self.joy = msg

    def update(self, event):

        if self.joy is None:
            return

        # Compute eff goal
        dx = self.joy.axes[4]
        dy = self.joy.axes[0]
        dz = self.joy.axes[1]

        p = self.p + DAMP * np.array([dx, dy, dz])

        self.problem.get_task_maps()['p2l'].end_point = p
        self.problem.start_state = self.q

        # Solve
        q = self.solver.solve()[0]
        publish_pose(q, self.problem)

        # Pack/publish joint state
        msg_joint_state = JointState()
        msg_joint_state.header.stamp = rospy.Time.now()
        msg_joint_state.position = q
        msg_joint_state.velocity = (q - self.q) / DT
        self.pub['joint_state'].publish(msg_joint_state)

        # Marker pub
        msg_mark = MarkerMsg(p)
        self.pub['marker'].publish(msg_mark)

        # Set new as old
        self.q = q
        self.p = p


if __name__ == '__main__':
    rospy.init_node('example_point_to_line_node')
    exo.Setup.init_ros()
    rospy.Timer(rospy.Duration(DT), Example().update)
    signal.signal(signal.SIGINT, sig_int_handler)
    rospy.spin()
