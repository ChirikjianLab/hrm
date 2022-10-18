#!/bin/bash

set -e

if [[ -d "$HOME"/tmpHRM ]]; then
  echo "tmpHRM exists already, skip creating it"
else
  echo "creating tmpHRM folder"
  mkdir "$HOME"/tmpHRM
  cd "$HOME"/tmpHRM || exit
fi

buildAndInstall() {
  mkdir build && cd build || exit
  cmake -G Ninja ../"$1"
  ninja && sudo ninja install
  cd .. && rm -rf build && rm -rf "$1"
}


# Build essentials
sudo apt -y update
sudo apt -y upgrade
sudo apt -y install g++ cmake pkg-config libeigen3-dev libboost-dev libgtest-dev ninja-build
export CXX=g++

# Install dependency from deb packages manager
sudo apt -y install libccd-dev libgmp-dev libmpfr-dev libompl-dev iburdfdom-dev libtinyxml2-dev

# Install dependencies from source
# fcl 0.6
git clone https://github.com/flexible-collision-library/fcl.git
srcDir="fcl"
buildAndInstall "$srcDir"

# cgal 5.2
git clone https://github.com/CGAL/cgal.git
srcDir="cgal"
buildAndInstall "$srcDir"

# kdl build and install
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
srcDir="orocos_kinematics_dynamics"
cd "$srcDir" || exit
srcDir="orocos_kdl"
buildAndInstall "$srcDir"
cd "$HOME"/tmpHRM || exit

sudo rm -r "$HOME"/tmpHRM
