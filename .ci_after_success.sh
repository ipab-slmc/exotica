#!/bin/bash
# CI after_success script: builds a debian and uploads it
#
# author: Wolfgang Merkt

# Only build a deb for the master branch
#if [ $TRAVIS_BRANCH -eq "master" ]
#then
  VERSION=`date +%Y.%m.%d-%H-%M`

  mkdir -p /create_deb/DEBIAN
  echo '
Package: exotica
Version: '"$VERSION"'
Architecture: all
Maintainer: IPAB-SLMC, The University of Edinburgh
Priority: optional
Homepage: http://github.com/openhumanoids/exotica
Description: This is a preliminary debian package for EXOTica.
 Please note that this file is for build-server purposes only where dependencies are manually satisified. It will likely be broken on other systems.
' > /create_deb/DEBIAN/control

  mkdir -p /create_deb/opt/ros/indigo/include
  mkdir -p /create_deb/opt/ros/indigo/lib
  mkdir -p /create_deb/opt/ros/indigo/share
  cp -R ~/catkin_ws/install/include/ /create_deb/opt/ros/indigo/
  cp -R ~/catkin_ws/install/lib/ /create_deb/opt/ros/indigo/
  cp -R ~/catkin_ws/install/share/ /create_deb/opt/ros/indigo/

  dpkg-deb --build /create_deb /exotica.deb
#fi
