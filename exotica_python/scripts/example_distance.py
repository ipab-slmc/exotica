#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import *
import math

exo.Setup.initRos()
ompl=exo.Setup.loadSolver('{exotica}/resources/configs/example_distance.xml')
ompl.getProblem().getScene().loadSceneFile('{exotica}/resources/scenes/example_distance.scene')
sc=ompl.getProblem().getScene()

dt=0.01
t=0.0
while not is_shutdown():
    sc.Update([math.sin(t/2.0)*0.5]*7)
    p=sc.getCollisionDistance(False)
    sc.getSolver().publishFrames()
    sc.publishProxies(sc.getCollisionDistance(False))
    t=t+dt
    sleep(dt)
