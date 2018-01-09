**************
Initialisation
**************

Now we have installed and set up EXOTica, we can now look at solving motion plans. 
To do this, we must first initialise the solver. EXOTica can be added to a new or existing project;
here we will start with a new project. 

Two primary steps need to be fulfilled when using EXOTica:
Initialisation and Usage. Initialisation is required to set up the
problem and the solver. The usage is where the problem is solved and the resulting motion plan is
used/processed.

The solving and usage of EXOTica is taken care of in your python or cpp code you can initialise your problem here too, or this can be done via a separate XML file. XML tends to be preferred, 
as it keeps your code clean and can be changed more easily.
We will look at how to achieve each of these in the next section, but first take note of 
where the initialisers are stored.

Initialisation options for all problems and solvers can be found in the
source code in the following locations:

::

    Problems: exotica/exotica/init/<ProblemName>.in

    Solvers: exotica/exotations/solvers/<SolverName>/init/<SolverName>.in
   

These ``.in`` files are laid out as follows:

When we look at a problem initialiser such as UnconstrainedEndPoseProblem initialiser 
(exotica/exotica/init/UnconstrainedEndPoseProblem.in), we see a list of parameters 
that can be set when we initialise the problem. 

.. code:: xml

    <!--UnconstrainedEndPoseProblem init options--> 

    extend <exotica/PlanningProblem>
    Optional Eigen::VectorXd W = Eigen::VectorXd();
    Optional Eigen::VectorXd Rho = Eigen::VectorXd();
    Optional Eigen::VectorXd Goal = Eigen::VectorXd();
    Optional Eigen::VectorXd NominalState = Eigen::VectorXd();

In this case, all the parameters are optional and will have default values assigned to them. 
We can change these parameters during initialisation if required. 

All problem initialisers also extend the ``exotica/PlanningProblem`` initialiser, which adds extra
parameters. This is seen below:

.. code:: xml

    include <exotica/SceneInitializer>
    extend <exotica/Object>
    Required exotica::Initializer PlanningScene; # SceneInitializer
    Optional std::vector<exotica::Initializer> Maps = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd StartState = Eigen::VectorXd();
    Optional double StartTime = 0;

This requries us to specify as scene in which the solver will operate, as well as start states and
task maps (which we will look at in a later section). In all, after extending both the 
PlanningSceneInitializer and the Object initialiser (which takes a name and debug argument), 
the whole initialiser for our UnconstrainedEndPoseProblem looks like this: 

.. code:: xml

    Required std::string Name;
    Optional bool Debug = false;
    Required exotica::Initializer PlanningScene; # SceneInitializer
    Optional std::vector<exotica::Initializer> Maps = std::vector<exotica::Initializer>();
    Optional Eigen::VectorXd StartState = Eigen::VectorXd();
    Optional double StartTime = 0;
    Optional Eigen::VectorXd W = Eigen::VectorXd();
    Optional Eigen::VectorXd Rho = Eigen::VectorXd();
    Optional Eigen::VectorXd Goal = Eigen::VectorXd();
    Optional Eigen::VectorXd NominalState = Eigen::VectorXd();

This shows us that the very least that is required to initialise a problem is a name for the problem
and the name of the scene that the problem operates in. Other problems are also laid out in a similar fashion. The optional elements offer a powerful customisation tool for out motion planner. 

Now we have seen the layout of the initialisers, the next step is to use these options in initialising EXOTica, 
which can be done via `XML <XML.html>`__ or `C++ <Manual-Initialisation.html>`__. We will first look at XML initialisation.
