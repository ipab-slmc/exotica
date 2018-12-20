#!/usr/bin/env python
# This is a workaround for liburdf.so throwing an exception and killing
# the process on exit in ROS Indigo.

import subprocess
import os
import sys
import signal

roslaunch = ['CppCore',
             'CppInitGeneric',
             'CppInitManual',
             'CppInitXML',
             'CppPlanAICO',
             'CppPlanIK',
             'CppPlanIKFreebase',
             'CppPlanOMPL',
             'CppPlanOMPLFreebase',
             'PythonAttachDemo',
             'PythonCollisionDistance',
             'PythonPlanAICO',
             'PythonPlanAICOEffVelocity',
             'PythonPlanAICOJointVelocityLimit',
             'PythonPlanAICOTrajectory',
             'PythonPlanBayesianIK',
             'PythonPlanIK',
             'PythonPlanIKEffAxisAlignment',
             'PythonPlanIKPointToLine',
             'PythonPlanIKTrajectory',
             'PythonPlanIKValkyrie',
             'PythonPlanIKWithJntSmooth',
             'PythonPlanLazyPRM',
             'PythonPlanLevenbergMarquardtIK',
             'PythonPlanOMPL',
             'PythonPlanOMPLProjections',
             'PythonPlanPRM',
             'PythonPlanSphereCollision',
             'PythonPlanTimeIndexedSampling',
             'PythonPlanTimeIndexedSamplingFreebase',
             ]

rosrun = ['example.py',
          'example_aico_noros.py',
          'example_fk.py',
          'example_ik_noros.py',
          'example_ompl_noros.py',
          'example.py',
          'IK',
          ]

process = None
isStopping = False
results = {}


def sig_int_handler(sig, frame):
    global isStopping
    global process
    if not isStopping and process is not None:
        isStopping = True
        process.send_signal(sig)
    else:
        raise KeyboardInterrupt


signal.signal(signal.SIGINT, sig_int_handler)

for example in rosrun:
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\nRunning '+example+'\n')
    isStopping = False
    process = subprocess.Popen(
        ['rosrun', 'exotica_examples', example], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while process.poll() == None:
        print(process.stdout.readline())
        sys.stdout.flush()
    print(process.stderr.read())
    results[example] = process.poll()

for example in roslaunch:
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\nRunning '+example+'\n')
    isStopping = False
    process = subprocess.Popen(['roslaunch', 'exotica_examples', example +
                                '.launch'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while process.poll() == None:
        print(process.stdout.readline())
        sys.stdout.flush()
    print(process.stderr.read())
    results[example] = process.poll()

print('All examples have been executed:')
print(results)
