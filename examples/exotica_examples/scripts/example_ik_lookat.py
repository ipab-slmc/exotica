#!/usr/bin/env python
import rospy
import pyexotica as exo
import numpy as np
import signal
from pyexotica.publish_trajectory import publishPose, sigIntHandler
from time import sleep
import task_map_py
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray as FloatArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA as Color

GREEN = Color()
GREEN.r=0.0
GREEN.g=1.0
GREEN.b=0.0
GREEN.a=1.0

DT = 1.0/100.0 # 100 HZ
DAMP = 0.005

def MarkerMsg(p):
    marker = Marker()
    marker.type=marker.SPHERE
    marker.id=0
    marker.action=marker.ADD
    marker.header.frame_id='exotica/world_frame'
    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = p[2]
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.pose.orientation.w=1.0
    marker.color=GREEN
    return marker

class Example(object):

    def __init__(self):

        # Set init variables
        self.q = np.array([0.0] * 7)
        self.joy = None

        # Setup exotica
        self.solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/ik_lookat.xml')
        self.problem = self.solver.getProblem()

        # Setup ros publisher
        self.pub = {}
        self.pub['marker'] = rospy.Publisher('point', Marker, queue_size=1)
        
        # Setup ros subscriber
        self.sub = {}
        self.sub['joy'] = rospy.Subscriber('joy', Joy, self.callback)
        
    def callback(self, msg):
        self.joy = msg

    def update(self, event):
        if self.joy is None: return

        # Get look_at_target
        look_at_target = self.problem.getTaskMaps()['LookAt'].getLookAtTarget()

        # Update point position
        dx = self.joy.axes[4]
        dy = self.joy.axes[0]
        dz = self.joy.axes[1]
        look_at_target += DAMP*np.array([dx, dy, dz])

        # Set new look at target into world frame
        self.problem.getScene().attachObjectLocal('LookAtTarget', '', exo.KDLFrame(look_at_target))

        # Set start state
        self.problem.startState = self.q

        # Publish point as marker
        self.pub['marker'].publish(MarkerMsg(look_at_target))

        # Solve and publish
        q = self.solver.solve()[0]
        publishPose(q, self.problem)

        # Set new as old
        self.q = q

if __name__=='__main__':
    rospy.init_node('example_ik_lookat_node')
    exo.Setup.initRos()
    signal.signal(signal.SIGINT, sigIntHandler)
    rospy.Timer(rospy.Duration(DT), Example().update)
    rospy.spin()
