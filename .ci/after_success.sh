#!/bin/bash
# CI after_success script: builds a debian and uploads it
#
# author: Wolfgang Merkt

VERSION=`date +%Y.%m.%d-%H-%M`

CI_ROS_DISTRO="kinetic"

mkdir -p ~/create_deb/DEBIAN
echo '
Package: exotica
Version: '"$VERSION"'
Architecture: all
Maintainer: IPAB-SLMC, The University of Edinburgh
Priority: optional
Homepage: http://github.com/ipab-slmc/exotica
Description: This is a preliminary debian package for EXOTica.
 Please note that this file is for build-server purposes only where dependencies are manually satisified. It will likely be broken on other systems.
' > ~/create_deb/DEBIAN/control

mkdir -p ~/create_deb/opt/ros/$CI_ROS_DISTRO/include
mkdir -p ~/create_deb/opt/ros/$CI_ROS_DISTRO/lib
mkdir -p ~/create_deb/opt/ros/$CI_ROS_DISTRO/share
cp -R ~/catkin_ws/install/include/ ~/create_deb/opt/ros/$CI_ROS_DISTRO/
cp -R ~/catkin_ws/install/lib/ ~/create_deb/opt/ros/$CI_ROS_DISTRO/
cp -R ~/catkin_ws/install/share/ ~/create_deb/opt/ros/$CI_ROS_DISTRO/

dpkg-deb --build ~/create_deb $TRAVIS_BUILD_DIR/exotica.deb
