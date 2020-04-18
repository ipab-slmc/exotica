# Instructions for ROS Indigo (Ubuntu 14.04)

We retired support for ROS Indigo in September 2018. For using 14.04, a number of manual changes are required:
* For Eigen, we recommend system-installing 3.2.10 or newer, for which we provide a [Debian](http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb).
* gcc >[4.9](https://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu) -- the 14.04 system-installed 4.8.4 won't work. On Ubuntu 16.04, you can use the system-provided gcc/g++.
* For compiling ``fcl_catkin``, you need to add a PPA for ``libccd-dev``.
