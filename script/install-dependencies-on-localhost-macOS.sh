#!/bin/bash

set -e

if [ `id -u` == 0 ]; then
    SUDO=
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

# Install Homebrew if not already installed
which brew >/dev/null || /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Update brew and upgrade packages
brew update
brew upgrade

# Install build essentials
brew install gcc cmake pkg-config eigen boost gtest ninja wget git

# Set the C++ compiler to use
export CXX=g++

# Install dependencies from brew
brew install libccd gmp mpfr cgal urdfdom tinyxml2

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