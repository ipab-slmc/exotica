XML Initialisation
==================

This section of the tutorial will demonstrate how to initialise EXOTica
using XML and the related C++ code that is needed to parse the XML file.
We will be using the XML file under the
`IK\_Solver <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/resources/ik_solver_demo.xml>`__
file and is shown below:

.. code:: xml

    <?xml version="1.0" ?>
    <IKSolverDemoConfig>

      <IKsolver Name="MySolver">   <!-- Motion solver definition -->
        <MaxIt>1</MaxIt>
        <MaxStep>0.1</MaxStep>
        <Tolerance>1e-5</Tolerance>
        <Alpha>1.0</Alpha>
        <C>1e-3</C>
      </IKsolver>

      <UnconstrainedEndPoseProblem Name="MyProblem"> <!-- Problem definition -->

        <PlanningScene>
          <Scene Name="MyScene"> <!-- Kinematic scene -->
            <PlanningMode>Optimization</PlanningMode>
            <JointGroup>arm</JointGroup>
          </Scene>
        </PlanningScene>
        
        <Maps>
          <EffPosition Name="Position">
            <Scene>MyScene</Scene>
            <EndEffector>
                <Frame Link="lwr_arm_6_link" />
            </EndEffector>
          </EffPosition>
        </Maps>

        <!-- Problem parameters: tolerance and per joint weighting -->

        <W> 7 6 5 4 3 2 1 </W>
      </UnconstrainedEndPoseProblem>

    </IKSolverDemoConfig>

Code Explained
--------------

| As seen above: \* the ``IKSolver`` is initialised first \* its
  parameters (e.g. ``MaxIt``,\ ``MaxStep``) are set
| \* a problem (``UnconstrainedEndPoseProblem``) is specified and the
  necessary parameters are input. - Within the problem, a ``scene`` and
  a ``map`` are initialised in addition to the problem parameters. -
  \*\*(A task map needs to be initialised in most problems, but not in
  the Sampling Problem)

Solver Setup
~~~~~~~~~~~~

The solver is initialised by identifying the solver and giving it a name
(to be used later): ``<IKsolver Name="MySolver">`` Solver options are
then specified:

.. code:: xml

        <MaxIt>1</MaxIt>
        <MaxStep>0.1</MaxStep>
        <Tolerance>1e-5</Tolerance>
        <Alpha>1.0</Alpha>
        <C>1e-3</C>

The options associated with each solver can be found in the init files
(detailed on the `previous
page <https://github.com/openhumanoids/exotica/wiki/Initialisation>`__)
and the function of each can be found in the literature.

Problem Setup
~~~~~~~~~~~~~

Initialisation of the problem is wrapped in the XML tag named after the
problem. Here we have the ``UnconstrainedEndPoseProblem`` (examples
covered later: ``SamplingProblem``,\ ``UnconstrainedTimeIndexProblem``).

Inside this problem, we have the: \* ``PlanningScene`` \* ``Maps`` \*
``W``.

Planning Scene
^^^^^^^^^^^^^^

A ``PlanningScene`` always contains a ``Scene`` with a name (e.g.
``MyScene``), a ``PlanningMode`` and a ``JointGroup``. This joint group
corresponds to the planning group which was specified within the
`SRDF <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/resources/lwr_simplified.srdf#L12>`__
file, the part we are interested in is detailed below:

.. code:: xml

        ...
        <group name="arm">
            <chain base_link="lwr_arm_0_link" tip_link="lwr_arm_7_link" />
        </group>
        ...

We will use this ``group name`` for our initialisation:

.. code:: xml

        <JointGroup>arm</JointGroup>

This will direct EXOTica to the joint properties for the robot you are
using.

Maps
^^^^

When setting up ``Maps`` , the name of the scene you set up earlier is
passed in:

.. code:: xml

        <Scene>MyScene</Scene>

as well as an initialiser to direct EXOTica to the EndEffector link:

.. code:: xml

            <EndEffector>
                <Frame Link="lwr_arm_6_link" />
            </EndEffector>

*NOTE - the name of the end effector link must match that in the URDF
and SRDF files*

Weighting
^^^^^^^^^

Finally, we must set up the task ``W``. This simply consists of a vector
of descending order weights based on the number of DOF in your robot
(e.g. for a 3 DOF robot: ``<W> 3 2 1 </W>``)

Next Step
~~~~~~~~~

Now the XML initialisation has been completed, we can begin parsing it
to be used in EXOTica in the `next
step <https://github.com/openhumanoids/exotica/wiki/XML-Parsing>`__.
