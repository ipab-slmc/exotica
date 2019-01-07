**************
Initialization
**************

Now that we have installed and set up EXOTica, we begin looking at solving motion plans. 
To do this, we must first initialize our problems and solvers. 
EXOTica can be added to a new or existing project; here we will start with a new project. 

Two primary steps need to be fulfilled when using EXOTica:
Initialization and Usage. Initialization is required to set up the
problem and the solver. The usage is where the problem is solved.

The solving and usage of EXOTica is taken care of in your Python or C++ code. 
You can initialize your problem here too, or this can be done via a separate XML file. 
XML tends to be preferred, as it keeps your code clean and can be changed more easily without requiring recompilation (in the case of C++).
We will look at how to achieve each of these in the next section, but first take note of 
where the initializers are stored.

Finding Initializers
====================

Initialization options for problems, solvers, task maps, ets. can be found in the following location inside the respective packages:

::

    Problems: <package_name>/init/<initializer>.in
   
Initializer Layout
==================

These ``.in`` files are laid out as follows:

When we look at a problem initializer such as UnconstrainedEndPoseProblem initializer 
(exotica/exotica/init/UnconstrainedEndPoseProblem.in), we see a list of parameters 
that can be set when we initialize the problem. 

.. code-block:: shell

    // UnconstrainedEndPoseProblemInitializer

    extend <exotica/PlanningProblem>
    Optional std::vector<exotica::Initializer> Cost = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd W = Eigen::VectorXd();
    Optional Eigen::VectorXd NominalState = Eigen::VectorXd();

In this case, all the parameters are optional and will have default values assigned to them. 
We can change these parameters during initialization if required. 

All problem initializers also extend the ``exotica/PlanningProblem`` initializer, which adds extra
parameters. This is seen below:

.. code-block:: shell

    // PlanningProblemInitializer

    include <exotica/SceneInitializer>
    extend <exotica/Object>
    Required exotica::Initializer PlanningScene; # SceneInitializer
    Optional std::vector<exotica::Initializer> Maps = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd StartState = Eigen::VectorXd();
    Optional double StartTime = 0;
    Optional int DerivativeOrder = -1;

This requires us to specify the scene in which the solver will operate, as well as start states and
task maps (which we will look at later). In all, after extending both the 
PlanningSceneInitializer and the Object initializer (which takes a name and debug argument), 
the whole initializer for our UnconstrainedEndPoseProblem looks like this: 

.. code-block:: xml

    Required std::string Name;
    Required exotica::Initializer PlanningScene; # SceneInitializer
    Optional bool Debug = false;
    Optional std::vector<exotica::Initializer> Maps = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd StartState = Eigen::VectorXd();
    Optional double StartTime = 0;
    Optional int DerivativeOrder = -1;
    Optional std::vector<exotica::Initializer> Cost = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd W = Eigen::VectorXd();
    Optional Eigen::VectorXd NominalState = Eigen::VectorXd();

This shows us that the very least that is required to initialize a problem is a name for the problem
and the planning scene that the problem operates in. Other problems are also laid out in a similar fashion. The optional elements offer a powerful customisation tool for out motion planner.
