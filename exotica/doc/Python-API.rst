***************
Python Bindings
***************

By default, the Python bindings will be compiled for 2.7. In order to build for different versions of Python you can specify ``PYBIND_PYTHON_EXECUTABLE`` in the additional CMake arguments of your catkin workspace:

.. code-block:: bash

    catkin config --cmake-args -DPYBIND_PYTHON_EXECUTABLE=/usr/bin/python3

.. NB: This is deactivated/not included in the toctree until the segfault upon loading the Python module is resolved - this prevents auto-generation.

.. automodule: : pyexotica