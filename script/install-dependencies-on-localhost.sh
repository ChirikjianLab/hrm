#!/bin/bash

set -e

if [ `id -u` == 0 ]; then
    SUDO=
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y install lsb-release
else
    SUDO="sudo -H"
fi

if [[ -d "$HOME"/tmpHRM ]]; then
  echo "tmpHRM exists already, skip creating it"
else
  echo "creating tmpHRM folder"
  mkdir "$HOME"/tmpHRM
fi

buildAndInstall() {
  mkdir build && cd build || exit
  cmake -G Ninja ../"$1"
  ninja && ${SUDO} ninja install
  cd .. && rm -rf build && rm -rf "$1"
}


# Build essentials
${SUDO} apt -y update
${SUDO} apt -y upgrade
${SUDO} apt -y install g++ cmake pkg-config libeigen3-dev libboost-dev libboost-filesystem-dev libgtest-dev ninja-build wget git
export CXX=g++

# Install dependency from deb packages manager
${SUDO} apt -y install libccd-dev libgmp-dev libmpfr-dev libcgal-dev liburdfdom-dev libtinyxml2-dev

# Install dependencies from source
cd "$HOME"/tmpHRM || exit

# fcl 0.6.1
wget -O - https://github.com/flexible-collision-library/fcl/archive/refs/tags/v0.6.1.tar.gz | tar zxf -
srcDir="fcl-0.6.1"
buildAndInstall "$srcDir"

# ompl 1.5.2
wget -O - https://github.com/ompl/ompl/archive/refs/tags/1.5.2.tar.gz | tar zxf -
srcDir="ompl-1.5.2"
buildAndInstall "$srcDir"

# cgal 5.5.1
wget -O - https://github.com/CGAL/cgal/archive/refs/tags/v5.5.1.tar.gz | tar zxf -
srcDir="cgal-5.5.1"
buildAndInstall "$srcDir"

# kdl build and install
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
srcDir="orocos_kinematics_dynamics"
cd "$srcDir" || exit
srcDir="orocos_kdl"
buildAndInstall "$srcDir"
cd "$HOME"/tmpHRM || exit

${SUDO} rm -r "$HOME"/tmpHRM
