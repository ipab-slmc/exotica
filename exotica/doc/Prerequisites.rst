*************
Prerequisites
*************

URDF File
=========

As detailed in the explanation of EXOTica, the library extracts the
dimensions and details of the robot from the robot's URDF configuration
file. The URDF file for the
`LWR\_simplified <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/resources/lwr_simplified.urdf>`__
kuka arm included in the source files will be used throughout these
tutorials.

SRDF File
=========

In addition to the URDF file, an SRDF file is also required. This
contains semantic information about the robot extracted from the URDF.
EXOTica extracts joint dimensions, limits, DH parameters and self
collision matrices from the SRDF file. Follow the MoveIt! setup
assistant tutorial to generate an SRDF file. The SRDF file for the
`LWR\_simplified <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/resources/lwr_simplified.srdf>`__
kuka arm included in the source files will be used throughout these
tutorials.

We will direct EXOTica to the URDF and SRDF files in the ROSLaunch
section.

CMakeLists.txt & package.xml
============================

Before using EXOTica, make sure the files you are using to code are
referenced in CMakeLists.txt and package.xml and correctly linked to
EXOTica and its dependencies. An example of this can be seen below:

[Will add proper CMakeList.txt instructions later]
