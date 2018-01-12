******************
Quickstart: C++
******************

For this quick tutorial, we will be using the `ik_minimal.cpp <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/src/ik_minimal.cpp>`_
in the examples folder. This uses the lwr_simplified robot in the examples. 

Running Example Code 
====================

To run this code and start producing motion plans, start roscore then run:

.. code-block:: shell

    rosrun exotica_examples IK

Other demos have associated roslaunch files. See the `demos <Installation.html>`_ 
section for more information. 

Expected output
===============

Running the code will produce a result similar to the following in the terminal:

.. code-block:: shell

    [ INFO] [1505929027.003072480]: Loading robot model 'lwr'...
    
    Publishing IK
    
    Finished solving in 0.000280003s Solution [ -6.18621023e-15  -9.09070542e-02  -8.66069432e-15   9.26337047e-02 -1.44344905e-14  -1.00000000e-01   0.00000000e+00]
    
    Finished solving in 0.000178323s Solution [-0.01107742 -0.07860809  0.0059596   0.1926337   0.00494248 -0.07840889 0.]
    ...

.. rubric:: CODE

This displays the joint angles for each of the 7 joints on our example robot after the time it took to solve the problem.

Now that we have executed the example and seen the result, let's look at what happening in the code. Here we are using the 
`ik_minimal.cpp <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/src/ik_minimal.cpp>`_ example. 
This is shown below:

.. code-block:: cpp

    #include <exotica/Exotica.h>

    using namespace exotica;

    int main(int argc, char **argv)
    {
        {
            MotionSolver_ptr solver = XMLLoader::loadSolver("{exotica_examples}/resources/configs/example.xml");
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

After including the Exotica header file and setting the namespace, we enter the main function and instantiate a MotionSolver_ptr: 

.. code-block:: cpp

    ...
    MotionSolver_ptr solver = XMLLoader::loadSolver("{exotica_examples}/resources/configs/example.xml");
    ...

This sets up our motion planning solver and gets the problem ready to be solved. To the solver we assign the contents of an XML 
file, which is parsed by Exotica (``XMLLoader::loadSolver("{exotica_examples}/resources/configs/example.xml")``). 
We will look in more detail into XML initialization in a later tutorial, but for now we will go through some basics. 

Quick XML Initialization
========================

The XML file from which the parser loads is `included <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/resources/configs/example.xml>`_ 
in the ``exotica_examples`` folder. We can see a copy of it below:

.. code-block:: xml

    <?xml version="1.0" ?>
    <ExampleConfig>

    <IKsolver Name="MySolver">
    <MaxIt>100</MaxIt>
    <MaxStep>0.1</MaxStep>
    <Tolerance>1e-5</Tolerance>
    <Alpha>1.0</Alpha>
    <C>1e-3</C>
    </IKsolver>

    <UnconstrainedEndPoseProblem Name="ExampleProblem">
    <PlanningScene>
        <Scene>
        <JointGroup>arm</JointGroup>
        <URDF>{exotica_examples}/resources/robots/lwr_simplified.urdf</URDF>
        <SRDF>{exotica_examples}/resources/robots/lwr_simplified.srdf</SRDF>
        </Scene>
    </PlanningScene>
    <Maps>
        <EffPosition Name="Position">
        <EndEffector>
            <Frame Link="lwr_arm_7_link" BaseOffset="0.5 0 0.5 0 0 0 1"/>
        </EndEffector>
        </EffPosition>
    </Maps>
    <W> 7 6 5 4 3 2 1 </W>
    <StartState>0 0 0 0 0 0 0</StartState>
    <NominalState>0 0 0 0 0 0 0</NominalState>
    </UnconstrainedEndPoseProblem>

    </ExampleConfig>

We can alter the properties of the solver and the problem in their respective XML tags. We can alter 
the goal tolerance for example by altering the value in the ``<Tolerance>1e-5</Tolerance>`` tag. 
Altering the ``StartState`` will change initial position of the edited joints, changing the motion plan.

Changing the XML tags will alter the behaviour of the motion planner when we run the ``ik_minimal`` again. 

As we move further into the tutorials, we will see the effects of altering various properties and adding 
more task maps to the problem. 

Solving Motion Plan
===================

After we have loaded the solver, we can solve the problem and display the result: 

.. code-block:: cpp

    solver->Solve(solution);

    HIGHLIGHT("Finished solving in " << timer.getDuration() << "s. Solution [" << solution << "]");

``solver`` is the name of the motion solver which we instantiated earlier. After creating it, 
solving is trivial; simply use ``solver->Solve(solution);``, passing in the ``Eigen::MatrixXd solution;`` 
container that was created before. 

The output of the motion plan is stored in this ``solution`` container, which we can then print to the 
terminal. This contains the sequence of joint angles which need to be achieved to reach the given goal. 
This matrix can be sent in sequence to your position controlled robot. 