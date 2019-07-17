***************
Python Bindings
***************

By default, the Python bindings will be compiled for 2.7. In order to build for different versions of Python you can specify ``PYBIND_PYTHON_EXECUTABLE`` in the additional CMake arguments of your catkin workspace:

.. code-block:: bash

    catkin config --cmake-args -DPYBIND_PYTHON_EXECUTABLE=/usr/bin/python3

.. automodule:: pyexotica
   :members:
   :undoc-members:
   :imported-members:
   :show-inheritance:

.. automodule:: exotica_ompl_solver_py
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: exotica_core_task_maps_py
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: exotica_ilqr_solver_py
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: exotica_ddp_solver_py
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: exotica_ilqg_solver_py
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: exotica_examples_py
    :members:
    :undoc-members:
    :show-inheritance:

.. Undocumented: exotica_scipy_solver
