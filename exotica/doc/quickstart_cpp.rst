******************
Quickstart: C++
******************

For this quick tutorial, we will be using the `ik_minimal.cpp <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/src/ik_minimal.cpp>`_
in the examples folder. This uses the lwr_simplified robot in the examples. 

Running Example Code 
====================

To run this code and start producing motion plans, start roscore then run:

.. code-block:: shell

    rosrun exotica_examples example_cpp_ik_minimal

Expected output
===============

Running the code will produce a result similar to the following in the terminal:

.. code-block:: shell

    Publishing IK
    
    Finished solving in 0.000280003s Solution [ -6.18621023e-15  -9.09070542e-02  -8.66069432e-15   9.26337047e-02 -1.44344905e-14  -1.00000000e-01   0.00000000e+00]
    
    Finished solving in 0.000178323s Solution [-0.01107742 -0.07860809  0.0059596   0.1926337   0.00494248 -0.07840889 0.]
    ...

.. rubric:: CODE

This displays the joint angles for each of the 7 joints on our example robot after the time it took to solve the problem.

Now that we have executed the example and seen the result, let's look at what happening in the code. Here we are using the 
`ik_minimal.cpp <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/src/ik_minimal.cpp>`_ example. 
This is shown below:

.. code-block:: cpp

    #include <exotica_core/exotica_core.h>

    using namespace exotica;

    int main(int argc, char **argv)
    {
        {
            MotionSolverPtr solver = XMLLoader::LoadSolver("{exotica_examples}/resources/configs/example_ik.xml");
            Eigen::MatrixXd solution;

            Timer timer;
            solver->Solve(solution);

            HIGHLIGHT("Finished solving in " << timer.getDuration() << "s. Solution [" << solution << "]");
        }
        Setup::Destroy();
    }


.. rubric:: CODE EXPLAINED

Quick Solver Setup
==================

After including the Exotica header file and setting the namespace, we enter the main function and instantiate a MotionSolverPtr: 

.. code-block:: cpp

    ...
    MotionSolverPtr solver = XMLLoader::loadSolver("{exotica_examples}/resources/configs/example_ik.xml");
    ...

This sets up our motion planning solver and gets the problem ready to be solved. To the solver we assign the contents of an XML 
file, which is parsed by EXOTica.

Quick XML Initialization
========================

The XML file from which the parser loads is `included <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/configs/example_ik.xml>`_ 
in the ``exotica_examples`` folder. We can see a copy of it below:

.. code-block:: xml

    <?xml version="1.0" ?>
    <IKSolverDemoConfig>

    <IKSolver Name="MySolver">
        <MaxIterations>1</MaxIterations>
    </IKSolver>

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

        <Cost>
        <Task Task="Position"/>
        </Cost>

        <StartState>0 0 0 0 0 0 0</StartState>
        <NominalState>0 0 0 0 0 0 0</NominalState>
        <W> 7 6 5 4 3 2 1 </W>
    </UnconstrainedEndPoseProblem>

    </IKSolverDemoConfig>

We can alter the properties of the solver and the problem in their respective XML tags.
Altering the ``StartState`` will change initial position of the edited joints, changing the motion plan.

Solving Motion Plan
===================

After we have loaded the solver, we can solve the problem and display the result: 

.. code-block:: cpp

    solver->Solve(solution);

    HIGHLIGHT("Finished solving in " << timer.getDuration() << "s. Solution [" << solution << "]");

``solver`` is the name of the motion solver which we instantiated earlier. After creating it, 
solving is trivial; simply use ``solver->Solve(solution);``, passing in the ``Eigen::MatrixXd solution;``.

The output of the motion plan is stored in the matrix, which we can then print to the terminal.
Since the XML specified an EndPose problem, the IKSolver computed a single robot configuration, returining a 1x7 matrix.