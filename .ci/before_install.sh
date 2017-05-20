#!/bin/bash
set -xe
wget https://bitbucket.org/eigen/eigen/get/3.2.6.zip
unzip 3.2.6.zip
cd eigen-eigen-c58038c56923
mkdir build
cd build
cmake ..
sudo make install
cd ../..