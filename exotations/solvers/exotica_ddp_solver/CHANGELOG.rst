^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica_ddp_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.1 (2020-11-17)
------------------

6.0.0 (2020-11-08)
------------------
* FDDP: More verbosity on NaN
* Use state transition function + derivative in place of hard-coded scheme
* BoxFDDP: Set default BoxQP options
* (Box)FDDP: Add GradientTolerance convergence criterion
* Add steplength and regularization evolution
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* FDDP: Only allocate if T changed
* FDDP: Use const-ref for warm-start
* Expose all internal data via const-ref getter
* Switch FDDP to LDLT decomposition
* Only contract with 2nd order derivatives if DynamicsSolver has them
* Pre-allocate, go over equations, convert to trajectory pre-allocation
* ControlLimitedDDPSolver: Clean-up Qxx, Qxu, Quu computation
* Move costs from Solver to Problem
* ControlLimitedDDPSolver: Add parameter for switching different BoxQP solvers
* Introduce thresholds for increase/decrease of regularization
* ControlLimitedDDPSolver: Add state regularization
* Re-use already allocated variables. Fixes `#678 <https://github.com/ipab-slmc/exotica/issues/678>`_
* Control-limited DDP: Use fixed, low regularization for BoxQP
* FDDP: Remove exception in ForwardPass
* FDDP: Replace exception to adapt regularisation
* FDDP: Use copy instead of reference to reset shooting nodes
* BoxDDP: Clamp in forward-pass
* Refactor FDDP to Abstract + Implementation
* Fix NX => NDX bugs
* Add BoxFDDP
* Correctly resize matrices, speed up AnalyticDDPSolver::BackwardPass by using LLT
* Use DynamicsSolver::ComputeDerivatives
* Add FeasibilityDrivenDDPSolver
* Contributors: Traiko Dinev, Wolfgang Merkt

5.1.3 (2020-02-13)
------------------

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* Add `exotica_ddp_solver`
* Contributors: Traiko Dinev, Wolfgang Merkt
