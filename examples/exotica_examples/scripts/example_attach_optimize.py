#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *
import numpy as np

def handleAttaching(i, x, scene):
    # Update state
    scene.Update(x)
    if i==0:
        # Reset object pose
        scene.attachObjectLocal('Item','',exo.KDLFrame([0.6, -0.3, 0.5, 0, 0, 0, 1]))
    if i==1:
        # Attach to end effector
        scene.attachObject('Item','lwr_arm_6_link')
    if i==2:
        # Detach
        scene.detachObject('Item')

def handleAttaching2(i, x, scene, sceneOrig):
    # Update state
    scene.Update(x)
    if i==0:
        # Reset object pose
        scene.attachObjectLocal('Item','',exo.KDLFrame([0.6, -0.3, 0.5, 0, 0, 0, 1]))
    if i==1:
        # Attach to end effector
        scene.attachObjectLocal('Item','lwr_arm_6_link', sceneOrig.fk('Item'))
    if i==2:
        # Detach
        scene.attachObjectLocal('Item', '', sceneOrig.fk('Item'))

def linspace(a , b, step):
    ret=[]
    for i in range(len(a)):
        ret.append(np.linspace(a[i],b[i], step, True).tolist())
    return np.matrix(ret).transpose().tolist()

exo.Setup.initRos()
ik=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_manipulate_ik.xml')
ompl=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_manipulate_ompl.xml')

# Plan 3 end poses
ik.getProblem().setRho('Position1', 1)
ik.getProblem().setRho('Position2', 0)
goal_pose = [ik.solve()[0].tolist()]
ik.getProblem().setRho('Position1', 0)
ik.getProblem().setRho('Position2', 1)
goal_pose.append(ik.solve()[0].tolist())
goal_pose.append([0]*len(goal_pose[0]))
start_pose = [[0]*len(goal_pose[0]),goal_pose[0],goal_pose[1]]

# Plan 3 trajectories
solution = []
for i in range(0,3):
    aico=exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_manipulate_aico.xml')

    ompl.getProblem().startState = start_pose[i]
    ompl.getProblem().setGoalState(goal_pose[i])
    handleAttaching(i, start_pose[i], ompl.getProblem().getScene())
    handleAttaching2(i, start_pose[i], aico.getProblem().getScene(), ompl.getProblem().getScene())
    sol = ompl.solve().tolist()

    T=len(sol)
    aico.getProblem().T = T
    aico.getProblem().InitialTrajectory = sol
    for t in range(T):
        aico.getProblem().setGoal('Pose', sol[t], t)
        aico.getProblem().setRho('Pose', 0, t)
        aico.getProblem().setRho('Collision', 1e5, t)
        aico.getProblem().update(sol[-1],t)
    aico.getProblem().setRho('Collision', 0, -1)
    aico.getProblem().setRho('Pose', 1e3, -1)

    sol = aico.solve().tolist()
    solution.append(sol)

# Playback
dt=0.03
signal.signal(signal.SIGINT, sigIntHandler)
while True:
    try:
        for i in range(0,len(solution)):
            handleAttaching(i, start_pose[i], ompl.getProblem().getScene())
            for t in range(0,len(solution[i])):
                publishPose(solution[i][t], ompl.getProblem())
                sleep(dt)
    except KeyboardInterrupt:
        break
