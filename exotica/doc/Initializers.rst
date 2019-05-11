************
Initializers
************

Initialization of core components is done via _Initializers_. An
initializer is a collection of properties that each objects uses to
initialize itself. These could be either required or optional. Each core
class can get the value of these properties from its initializer, e.g. a
solver may require number of iterations or a convergence rate.

The initializers may be created manually (C++), from config files (XML)
or using wrappers (Python). These wrappers provide a useful abstractions
for creating and initializing EXOTica core components directly from
scripts or configuration files.

Creating a core class
=====================

A core class is a class derived from ``MotionSolver``,
``PlanningProblem``, ``TaskDefinition`` or ``TaskMap``. For the class to
be valid, it also have to inherit from ``Instantiable<>`` templated
class, e.g.:

.. code-block:: cpp

    class JointLimit: public TaskMap, public Instantiable<JointLimitInitializer>
    {
    public:
      virtual void Instantiate(JointLimitInitializer& init);
      ...
    }

In the case above, ``JointLimitInitializer`` is the template parameter
and the type used for initialization. You can then use the parameters of
``JointLimitInitializer`` to initialize the ``JointLimit`` class:

.. code-block:: cpp

    void JointLimit::Instantiate(const JointLimitInitializer& init)
    {
      double percent = init.SafePercentage;
      ...
    }

The property ``init.SafePercentage`` is of type ``double`` and it can be
directly used as such.

There are two types of initializers: specialized (e.g.
``exotica::JointLimitInitializer``) and generic
(``exotica::Initializer``). The generic initializer contains a
collection of loosely typed parameters exploiting the ``boost::any``
class. These initializers are used for parsing XML, python and other
data structures and they allow you to create an initializer on run time,
as opposed to compiling a specific datatype on compile time (this is
useful for reflection and creating wrappers for python and Matlab).

Specialized initializers are typeset and they are used within your class
to easily access the parameters a class needs to set itself up.
Conversion from generic to specialized initializers is handled by
EXOTica.

Creating specialized initializers
=================================

Each core class requires some properties to initialize successfully and
some properties that are optional. The way to declare properties, their
default values and whether are required or optional is to create a
specialized initializer type. This can be done by defining it in a
initializer file, e.g. ``init/JointLimit.in`` contains:

::

    extend <exotica_core/task_map>
    Required double SafePercentage;

The file name (excluding the extension ``.in``) defines the name of the
type: ``JointLimit.in``>>\ ``exotica::JointLimitInitializer``. The
syntax is as follows:

::

    // Comments start with "//" 
    extend <exotica_core/planning_problem> // Extend existing initializers (weak inheritance)
    Required double Tolerance; // Create a required property (no default value)
    Required Eigen::VectorXd W; // Create a required property with included type
    Required Initializer EndEffectorFrame; // Stores an initializer type, e.g. LimbInitilizer for defining spatial frames.
    Optional int T = 1; // Create an optional property (default value provided)

Compiling initializers
~~~~~~~~~~~~~~~~~~~~~~

To compile a package defining new initializers, add the following lines
to your CMakeLists.txt:

.. code-block:: cmake

    project(MyProjectName)

    # ...

    catkin_package(
    # ... catkin package stuff
    )

    # Add all names of defined initializers (excluding the *.in extension)
    AddInitializer(IKSolver)

    # Generate initializers
    GenInitializers()

    # ...

    # Add initializers dependency to your executable
    add_dependencies(MyExecutableName MyProjectName_initializers)

You may want to create initializers for core classes, but can also use
them for setting up other classes. See ``exotica::OMPLImpSolver`` for an
example.

Initializing EXOTica
====================

EXOTica can now be set up using initializers alone. You can either
create these manually or using loaders. See the ``exotica_examples``
package for examples of initializing exotica.

See ``exotica_examples/src/generic.cpp`` for an example of initializing
exotica from c++ without including any of the specialized headers.

See ``exotica_examples/src/xml.cpp`` for an example loading exotica from
XML. This is similar to the way EXOTica was initialized in the past. The
XML format has changed slightly though. See
``exotica_examples/resources/ik_solver_demo.xml`` for an example of the
new format.
