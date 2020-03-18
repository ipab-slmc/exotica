from __future__ import print_function
import subprocess

output = subprocess.check_output("python -c 'import pyexotica as exo ; initializers = exo.Setup.get_initializers()'", shell=True)

if "Skipping" in output:
    print(output)
    assert "Skipping" not in output
