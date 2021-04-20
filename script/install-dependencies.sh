#!/bin/sh

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

# libcdd build and install
git clone git@github.com:danfis/libccd.git
srcDir="libccd"
buildAndInstall "$srcDir"

# fcl bulid and install
git clone git@github.com:flexible-collision-library/fcl.git
srcDir="fcl"
buildAndInstall "$srcDir"

# boost install
sudo add-apt-repository ppa:mhier/libboost-latest
sudo apt update
sudo apt install libboost-all-dev

# cgal install
sudo apt install libgmp-dev
sudo apt-get install libcgal-dev

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
