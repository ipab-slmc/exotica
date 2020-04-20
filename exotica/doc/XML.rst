******************
XML Initialization
******************

This section of the tutorial will demonstrate how to initialize EXOTica
using XML and the related C++ code that is needed to parse the XML file.
We will be using the XML file under the
`IK\_Solver <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/configs/example_ik.xml>`__
file and is shown below:

.. code-block:: xml

  <?xml version="1.0" ?>
  <IKSolverDemoConfig>
    <IKSolver Name="MySolver" MaxIterations="10"/>

    <UnconstrainedEndPoseProblem Name="MyProblem">
      <PlanningScene>
        <Scene>
          <JointGroup>arm</JointGroup>
          <URDF>{exotica_examples}/resources/robots/lwr_simplified.urdf</URDF>
          <SRDF>{exotica_examples}/resources/robots/lwr_simplified.srdf</SRDF>
        </Scene>
      </PlanningScene>
      
      <Maps>
        <EffFrame Name="Position">
          <EndEffector>
              <Frame Link="lwr_arm_6_link" LinkOffset="0 0 0 0.7071067811865476 -4.3297802811774664e-17  0.7071067811865475 4.3297802811774664e-17"/>
          </EndEffector>
        </EffFrame>
      </Maps>
      <StartState>0 0 0 0 0 0 0</StartState>
      <NominalState>0 0 0 0 0 0 0</NominalState>
      <W> 7 6 5 4 3 2 1 </W>
    </UnconstrainedEndPoseProblem>
  </IKSolverDemoConfig>


.. rubric:: CODE EXPLAINED

In the code we see:
* the initialization of the ``IKSolver`` and its parameters (e.g. ``MaxIterations``) set
* a problem (``UnconstrainedEndPoseProblem``) is specified and  necessary parameters input. 
- Within the problem, a ``PlanningScene`` and ``Maps`` are initialized in addition to the problem parameters. 

Let's look a little more closely into the solver and problem setup.


Solver Setup
============

The solver is initialized by identifying the solver and giving it a name
(which will be used later); here we use the name "MySolver" : ``<IKSolver Name="MySolver">`` 
Solver options are then specified:

.. code-block:: xml

      <MaxIterations>1</MaxIterations>
      <Tolerance>1e-5</Tolerance>
      <RegularizationRate>1e-3</RegularizationRate>

The parameters in this case are optional, but each solver has its own 
set of initialization parameters, as detailed on the `previous page <initialization.html>`__
and the function of each can be found in the literature.

Problem Setup
=============

Initialization of the problem is wrapped in the XML tag named after the
solver. Here we use the ``UnconstrainedEndPoseProblem``.

Inside this problem, we have the: 
* ``PlanningScene`` 
* ``Maps`` 
* Solver Parameters (e.g. ``StartState``, ``NominalState`` ``W``).

Planning Scene
==============

A ``PlanningScene`` always contains:
 * `URDF <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.urdf>`__
 * `SRDF <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.srdf>`__
 * ``JointGroup`` which corresponds to the planning group specified in the SRDF file, the part we are interested in is detailed below:

.. code-block:: xml

        ...
        <group name="arm">
            <chain base_link="lwr_arm_0_link" tip_link="lwr_arm_7_link" />
        </group>
        ...

We will use this ``group name`` for our initialisation:

.. code-block:: xml

        <JointGroup>arm</JointGroup>

This will direct EXOTica to the joint properties for the robot you are
using.

Maps
====

Maps refers to the ``task maps`` of a problem, they provide a mapping from configuration space to task space
which are useful for fulfilling several tasks, such as specifying goals and avoiding obstacles. 
You can read more about task maps in a `later section <Task_maps.html>`__ . 

For now we are only interested in reaching an end effector goal, so we will use the ``EffFrame`` task map, 
which allows us specify the name of the end effector from the URDF file, which will be the focus when we 
try to reach a an end effector goal, as we are doing here. 

.. code-block:: xml

      <Maps>
        <EffFrame Name="Position">
          <EndEffector>
              <Frame Link="lwr_arm_6_link" LinkOffset="0 0 0 0.7071067811865476 -4.3297802811774664e-17  0.7071067811865475 4.3297802811774664e-17"/>
          </EndEffector>
        </EffFrame>
      </Maps>

This specifies the maps we are using in the problem. Here we use only EffFrame, but you can add multiple task maps between the ``Maps`` tags. 

Within the EffFrame initialisation, we give the map a name, we specify the name of the link be be considered as the end effector as well as an optional offset distance. 

*NOTE - the name of the end effector link must match that in the URDF
and SRDF files*

Problem Parameters
==================

Finally, we setup the parameters of this problem. Parameters vary for each problem, but here we see the parameters ``W``, ``StartState`` and ``NominalState``, which
we set to the appropriate values. More information about these parameters can be found in the EXOTica chapter. 

The ``W`` vector weights the joints of your robot according to the cost of moving each one. 
This vector must be the same size as the number of the number of DOF of your robot. 

Next Step
~~~~~~~~~

Now the XML initialisation has been completed, we can begin parsing it
to be used in EXOTica in the `next
step <XML-Parsing.html>`__.
