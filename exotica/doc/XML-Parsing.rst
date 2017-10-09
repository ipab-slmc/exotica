***********
XML Parsing
***********

This part of the tutorial assumes that you have completed the previous
XML initialisation tutorial. If this is not the case, please go
`back <XML.html>`__ and complete it first.

In this section, we will create a parser in C++ to handle the XML file
we created last time.

This snippet of code shows how this is implemented in the
`XML.cpp <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/src/xml.cpp#L1-L15>`__
file in the EXOTica examples:

.. code:: c++

    #include <exotica/Exotica.h>
    #include <exotica/Problems/UnconstrainedEndPoseProblem.h>

    using namespace exotica;

    void run()
    {
        ros::NodeHandle nh_("~");

        Initializer solver, problem;

        std::string file_name;
        nh_.getParam("ConfigurationFile",file_name);

    XMLLoader::load(file_name,solver, problem);
    ...

Code Explained
--------------

Within this run function, we have the ``ros::nodehandle`` ``nh_``. We'll
use this to direct the parser to the XML file using ``ROS Params`` in
the ROSLaunch file [link to ROSLaunch section].

First, we'll need some blank initialisers for the parser to fill and
somewhere to store the name of the XML ``file_name``:

.. code:: c++

    Initializer solver, problem;

    std::string file_name;

Now to grab the file name of the XML file from the ROSLaunch file. In
this example we have used the name ``ConfigurationFile`` e.g.:

.. code:: xml

    <param name="ConfigurationFile" type="string" value="$(find exotica_examples)/resources/ik_solver_demo.xml" />

so we'll extract the value under this parameter and assign it to the
``file_name`` string using the ``ros::NodeHandle nh_``:

.. code:: c++

    nh_.getParam("ConfigurationFile",file_name);

The final step now we have all the components is to feed the
``file_name`` and empty initialisers into the XML loader:

.. code:: c++

    XMLLoader::load(file_name,solver, problem);

From here, the XML loader handles everything and will extract all the
initialisation parameters from the XML initialisation file. We're now
ready to move onto the final initialisation `step <Common-Initialisation-Step.html>`__.
