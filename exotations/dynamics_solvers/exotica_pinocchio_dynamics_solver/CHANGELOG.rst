^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica_pinocchio_dynamics_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.0 (2021-03-15)
------------------

6.0.2 (2020-11-23)
------------------
* Split derivatives into separate compile units to reduce memory consumption (7.6 GB peak to 6.1 GB peak) (`#729 <https://github.com/ipab-slmc/exotica/issues/729>`_)
* Contributors: Wolfgang Merkt

6.0.1 (2020-11-17)
------------------

6.0.0 (2020-11-08)
------------------
* Add state transition function, derivative.
* Support explicit Euler and semi-implicit Euler integration schemes
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* Fix Pinocchio packaging (for later versions)
* Add dynamics solver with gravity compensation
* Fix analytic derivatives and remove memory allocation during the computation
* Contributors: Wolfgang Merkt

5.1.3 (2020-02-13)
------------------

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* Added `exotica_pinocchio_dynamics_solver`
