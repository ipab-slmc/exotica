#!/usr/bin/env python
# This is a workaround for liburdf.so throwing an exception and killing
# the process on exit in ROS Indigo.
from __future__ import print_function

import subprocess
import os
import sys

cpptests = ['test_initializers',
         'test_maps', 'test_problems'
        ]

pytests = ['core.py',
         'valkyrie_com.py',
         'valkyrie_collision_check_fcl_default.py',
         'valkyrie_collision_check_fcl_latest.py',
         'collision_scene_distances.py',
         'test_continuous_collision_check.py',
         'scene_creation.py'
         #'issue225_kinematictree_memory_leak.py'
        ]

for test in cpptests:
    process=subprocess.Popen(['rosrun', 'exotica_examples', test],stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = process.stdout.readlines()
    print(''.join(output))
    if process.wait()!=0:
        print('Test '+test+' failed\n'+process.stderr.read())
        os._exit(1)

for test in pytests:
    print(">>> Starting test", test)
    process=subprocess.Popen(['rosrun', 'exotica_examples', test],stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = process.stdout.readlines()
    print(''.join(output))
    if output[-1][0:11]!='>>SUCCESS<<':
        print('Test '+test+' failed\n'+process.stderr.read())
        os._exit(1)
