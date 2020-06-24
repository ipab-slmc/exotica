from __future__ import print_function
import os
import subprocess
import unittest


class TestUnknownInitializerTypes(unittest.TestCase):
    def test_unknown_initializer_types(self):
        output = None
        if os.environ['ROS_PYTHON_VERSION'] == '2':
            output = subprocess.check_output(
                "/usr/bin/env python -c 'import pyexotica as exo\ninitializers = exo.Setup.get_initializers()'", shell=True)
        elif os.environ['ROS_PYTHON_VERSION'] == '3':
            output = subprocess.check_output(
                "/usr/bin/env python3 -c 'import pyexotica as exo\ninitializers = exo.Setup.get_initializers()'", shell=True)
        else:
            raise AssertionError("Unknown ROS_PYTHON_VERSION")

        if b"Skipping" in output:
            raise AssertionError(output)


if __name__ == '__main__':
    unittest.main()
