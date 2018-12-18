#!/usr/bin/env python

import pyexotica as exo
from pyexotica.publish_trajectory import sig_int_handler
from time import sleep
import signal
import math

exo.Setup.init_ros()
ompl = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/example_distance.xml')
sc = ompl.get_problem().get_scene()

dt = 0.01
t = 0.0
signal.signal(signal.SIGINT, sig_int_handler)
while True:
    try:
        sc.update([math.sin(t/2.0)*0.5]*7)
        p = sc.get_collision_distance(False)
        sc.get_solver().publish_frames()
        sc.publish_proxies(p)
        t = t+dt
        sleep(dt)
    except KeyboardInterrupt:
        break
