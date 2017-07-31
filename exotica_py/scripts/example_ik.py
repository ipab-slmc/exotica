#!/usr/bin/env python

import exotica_py as exo
import rospkg
from numpy import array
from numpy import matrix

from publish_trajectory import *

import rospy
import math

def findpkg(pkg):
    return rospkg.RosPack().get_path(pkg)

def figureEight(t):
    return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

(sol, prob)=exo.Initializers.loadXMLFull(findpkg('exotica_examples')+'/resources/ik_solver_demo.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

rospy.init_node('example_python_ik_publisher', anonymous=True)
dt=0.002
rate = rospy.Rate(1.0/dt)
t=0.0
q=array([0.0]*7)
print('Publishing IK')
while not rospy.is_shutdown():
    problem.setGoal('Position',figureEight(t))
    q = solver.solve(q)[0]
    publishPose(q, problem)
    rate.sleep()
    t=t+dt

