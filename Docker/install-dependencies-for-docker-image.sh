#!/bin/bash

# NOTE : This is meant to be called by Dockerfile only when building an image
# stop execution instantly. Also print the error location of the running code.
set -eE

if [[ -d "$HOME"/tmpHRM ]]; then
  echo "tmpHRM exists already, skip creating it"
else
  echo "creating tmpHRM folder"
  mkdir "$HOME"/tmpHRM
  cd "$HOME"/tmpHRM || exit
fi

buildAndInstall() {
  if [[ -d build ]]; then
    echo "build exists already, skip creating it"
  else
    echo "creating build folder"
    mkdir build && cd build || exit
  fi
  cmake -G Ninja ../"$1"
  ninja && sudo ninja install
  cd .. && rm -rf build && rm -rf "$1"
}

# Depend on Boost 1.71.0 as the default of Ubuntu 20.04

cd "$HOME"/tmpHRM
# libccd install from source
git clone https://github.com/danfis/libccd.git
srcDir="libccd"
buildAndInstall "$srcDir"

# fcl 0.6 install from source
git clone https://github.com/flexible-collision-library/fcl.git
srcDir="fcl"
buildAndInstall "$srcDir"

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

# gtest build and install
git clone https://github.com/google/googletest.git -b release-1.10.0
srcDir="googletest"
buildAndInstall "$srcDir"

 rm -r "$HOME"/tmpHRM
