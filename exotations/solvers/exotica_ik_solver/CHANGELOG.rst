^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica_ik_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.2 (2020-11-23)
------------------

6.0.1 (2020-11-17)
------------------

6.0.0 (2020-11-08)
------------------
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* Re-wrote the IK solver as regularised, weighted pseudo-inverse
  Re-wrote the IK solver from scratch:
  - It now implements a regularised and weighted Jacobian pseudo-inverse
  instead of solving a linear system.
  - The support for NominalState regularisation by appending a nominal
  state to the error vector was removed.
  - Adaptive regularisation and backtracking line-search result in better
  convergence and numerical behaviour.
  - Use of pre-allocation and Cholesky decomposition makes the solver fast
  :-).
  - MaxStep for interactive IK is still supported, however, only when
  MaxIterations == 1.
  - example_ik now converges in 5 iterations, previously 41.
* Contributors: Vladimir Ivan, Wolfgang Merkt

5.1.3 (2020-02-13)
------------------

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* A variety of small code improvements
* Contributors: Christian Rauch, Wolfgang Merkt
