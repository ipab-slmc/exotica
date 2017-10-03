#!/usr/bin/env python
# This is a workaround for liburdf.so throwing an exception and killing
# the process on exit in ROS Indigo.

import subprocess
import os
import sys

tests = ['core.py', 'valkyrie_com.py']

for test in tests:
    process=subprocess.Popen(['rosrun', 'exotica_python', test],stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = process.stdout.readlines()
    print(''.join(output))
    if output[-1][0:11]!='>>SUCCESS<<':
        print('Test '+test+' failed\n'+process.stderr.read())
        os._exit(1)
