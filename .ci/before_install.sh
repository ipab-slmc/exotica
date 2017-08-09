#!/bin/bash
set -xe
wget http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb
sudo dpkg -i libeigen3-dev.deb
rm libeigen3-dev.deb
# wget https://bitbucket.org/eigen/eigen/get/3.2.6.zip
# unzip 3.2.6.zip
# cd eigen-eigen-c58038c56923
# mkdir build
# cd build
# cmake ..
# sudo make install
# cd ../..