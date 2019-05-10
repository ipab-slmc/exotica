***********
XML Parsing
***********

This part of the tutorial assumes that you have completed the previous
XML initialization tutorial. If this is not the case, please go
`back <XML.html>`__ and complete it first.

In this section, we will create a parser in C++ to handle the XML file
we created last time. For Python, the procedure is very similar - refer
to the `Python examples <https://github.com/ipab-slmc/exotica/tree/master/exotica_examples/scripts>`_ .

This snippet of code shows how this is implemented in the
`xml.cpp <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/src/xml.cpp>`__
file in the EXOTica examples:

.. code-block:: c++

    #include <exotica_core/exotica_core.h>
    #include <exotica_core/problems/unconstrained_end_pose_problem.h>

    using namespace exotica;

    void run()
    {
        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));

        Initializer solver, problem;

        std::string file_name;
        Server::getParam("ConfigurationFile", file_name);

        XMLLoader::load(file_name, solver, problem);

    HIGHLIGHT_NAMED("XMLnode", "Loaded from XML");
    ...

.. rubric:: CODE EXPLAINED

EXOTica Server
==============

Firstly we use the exotica ``Server`` class to initialize the ROS node we will be using from now on.
This ``Server`` class will handle a lot of our ROS dealings from here on. 

We next get the filename of the ``Configuration File`` we need to parse. This argument is the name
of the XML file which we set up previously, which will have been specified in the roslaunch file under 
the parameter name ``ConfigurationFile``. 


We will look at setting up our roslaunch files in more detail `later <Setting-up-ROSlaunch.html>`__.

XML Loader
==========

Now we have the file name and have created some initializers in which to hold the ``solver`` and the ``problem``,
we can now do the actual XML loading. This is done with the XMLLoader:

.. code-block:: c++

    XMLLoader::load(file_name,solver, problem);

To which we pass the filename which we filled earlier and the two empty initializers. When these initializers
are returned they contain the initialization details which we input in the XML file. These are now ready for the 
next `step <Common-Initialization-Step.html>`__ which is common to the XML and coded initialization methods. 