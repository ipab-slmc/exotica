from __future__ import print_function
import subprocess
import os

output = None
if os.environ['ROS_PYTHON_VERSION'] == '2':
    output = subprocess.check_output("python -c 'import pyexotica as exo ; initializers = exo.Setup.get_initializers()'", shell=True)
elif os.environ['ROS_PYTHON_VERSION'] == '3':
    output = subprocess.check_output("python3 -c 'import pyexotica as exo ; initializers = exo.Setup.get_initializers()'", shell=True)
else:
    assert False

if b"Skipping" in output:
    print(output)
    assert b"Skipping" not in output
