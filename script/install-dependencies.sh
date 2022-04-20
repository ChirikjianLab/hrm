#!/bin/bash

# stop execution instantly. Also print the error location of the running code.
set -e

# TODO : temporary solution to resolve ssh priviledge. Set the owner of .ssh to the owner of docker, a.k.a, robot
# docker's username : robot
# docker's sudo password : robot
sudo chown -R robot:robot "$HOME"/.ssh
# Note: After exiting docker, you will need to reset the owner of .ssh on your localhost

eval "$(ssh-agent -s)"
ssh-add "$HOME"/.ssh/id_rsa

# Use ninja build system for fast speed
sudo apt-get install -y ninja-build

mkdir "$HOME"/tmpHRM
cd "$HOME"/tmpHRM || exit

buildAndInstall() {
  mkdir build && cd build || exit
  cmake -G Ninja ../"$1"
  ninja && sudo ninja install
  cd .. && rm -rf build && rm -rf "$1"
}

# Depend on Boost 1.71.0 as the default of Ubuntu 20.04

# libccd install from source
git clone https://github.com/danfis/libccd.git
srcDir="libccd"
buildAndInstall "$srcDir"

# fcl 0.6 install from source
git clone https://github.com/flexible-collision-library/fcl.git
srcDir="fcl"
buildAndInstall "$srcDir"

# cgal 5.2 install from source
sudo apt install libgmp-dev

git clone https://github.com/CGAL/cgal.git
srcDir="cgal"
buildAndInstall "$srcDir"

# ompl install
sudo apt-get install libompl-dev ompl-demos

# kdl build and install
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
srcDir="orocos_kinematics_dynamics"
cd "$srcDir" || exit
srcDir="orocos_kdl"
buildAndInstall "$srcDir"
cd "$HOME"/tmpHRM || exit

# gtest build and install
git clone https://github.com/google/googletest.git -b release-1.10.0
srcDir="googletest"
buildAndInstall "$srcDir"

# install deb package for urdfdom and tinmyxml
sudo apt install liburdfdom-dev
