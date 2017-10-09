Initialisation

Two primary steps need to be fulfilled when using EXOTica: Initialisation and Usage. Initialisation is required to set up the parameters of your robot and the problem to be solved. The usage is where the problem is solved and the resulting motion plan is used/processed.

The solving and usage of EXOTica must be hard-coded, but initialisation can be achieved through hard-coding or XML loading. Each of these methods are detailed below using the UnconstrainedEndPoseProblem included in the source code examples.

Initialisation options for all solvers and problems can be found in the source code in the following locations:

    Solvers: exotica/exotations/solvers/<SolverName>/init/<SolverName>.in

    Problems: exotica/exotica/init/<ProblemName>.in

```XML
<!--UnconstrainedEndPoseProblem init options--> 

Required Eigen::VectorXd W;
Optional Eigen::VectorXd Rho = Eigen::VectorXd();
Optional Eigen::VectorXd Goal = Eigen::VectorXd();
```
```XML
<!--IKSolver init options--> 

Optional double Tolerance = 1e-5;
Optional double Convergence = 0.0;
Optional int MaxIt = 50;
Optional double MaxStep = 0.02;
Optional double C = 0.0;
Optional double Alpha = 1.0;
```
The next step is to use these options in initialising EXOTica, which can be done via [XML](https://github.com/openhumanoids/exotica/wiki/XML) or [C++](https://github.com/openhumanoids/exotica/wiki/Manual-Initialisation). 