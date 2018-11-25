#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import sigIntHandler
from time import sleep
import signal
import math

exo.Setup.initRos()
ompl = exo.Setup.loadSolver('{exotica_examples}/resources/configs/example_distance.xml')
sc = ompl.getProblem().getScene()

dt = 0.01
t = 0.0
signal.signal(signal.SIGINT, sigIntHandler)
while True:
    try:
        sc.Update([math.sin(t/2.0)*0.5]*7)
        p = sc.getCollisionDistance(False)
        sc.getSolver().publishFrames()
        sc.publishProxies(p)
        t = t+dt
        sleep(dt)
    except KeyboardInterrupt:
        break
