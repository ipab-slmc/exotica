#!/usr/bin/env python
import rospy
import pyexotica as exo
import numpy as np
import signal
from pyexotica.publish_trajectory import publishPose, sigIntHandler
import task_map_py
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray as FloatArray

DT = 1.0/100.0 # 100 HZ
DAMP = 0.005

XLIM = [0.0, 0.75]
YLIM = [-0.5, 0.5]
ZLIM = [0, 1.0]

LIMITS = np.array([XLIM,\
                   YLIM,\
                   ZLIM])

class Example(object):

    def __init__(self):

        # Initialize general class attributes
        self.q = np.array([0.0]*7)
        self.joy = None
        self.eff = np.array([0.6, -0.1, 0.5, 0, 0, 0])
        

        # Setup EXOtica
        self.solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_eff_axis_alignment.xml')
        self.problem = self.solver.getProblem()

        # Setup ROS
        self.pub = {}
        self.pub['joint_state'] = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.pub['eff_state'] = rospy.Publisher('/eff_state', FloatArray, queue_size=1)

        self.sub = {}
        self.sub['joy'] = rospy.Subscriber('/joy', Joy, self.callback)
        
    def callback(self, msg):
        self.joy = msg

    def update(self, event):
        if self.joy is None: return

        # Compute eff goal 
        dx = self.joy.axes[7]
        dy = self.joy.axes[0]
        dz = self.joy.axes[1]
        da1 = self.joy.axes[4]
        da2 = self.joy.axes[3]

        eff = self.eff + DAMP * np.array([dx, dy, dz, 0, da1, da2])

        for i in xrange(3):
            if not LIMITS[i,0] <= eff[i] <= LIMITS[i,1]:
                eff[i] = self.eff[i]

        # eff (eul ang) -> dir vec
        direction = np.zeros(3)
        direction[0] = np.cos(eff[5])*np.cos(eff[4])
        direction[1] = np.sin(eff[5])*np.cos(eff[4])
        direction[2] = np.sin(eff[4])

        # Setup problem
        self.problem.setGoal('Position', eff[:3])
        self.problem.getTaskMaps()['Direction'].setDirection('lwr_arm_6_link', direction)
        self.problem.startState = self.q

        # Solve
        q = self.solver.solve()[0]
        publishPose(q, self.problem)

        # Pack/publish joint state
        msg_joint_state = JointState()
        msg_joint_state.header.stamp = rospy.Time.now()
        msg_joint_state.position = q
        msg_joint_state.velocity = (q - self.q) / DT
        self.pub['joint_state'].publish(msg_joint_state)

        # Pack/publish eff state
        msg_eff_state = FloatArray()
        msg_eff_state.data = eff
        self.pub['eff_state'].publish(msg_eff_state)
        
        # Set new as old
        self.q = q
        self.eff = eff

if __name__=='__main__':
    rospy.init_node('example_ik_eff_axis_alignment_node')
    exo.Setup.initRos()
    rospy.Timer(rospy.Duration(DT), Example().update)
    signal.signal(signal.SIGINT, sigIntHandler)
    rospy.spin()
