#!/usr/bin/env bash
# INSTALLER for Duckiebot RR interface
set -e
set -x

BASE_DIR=$(pwd)
RR_DIR="$BASE_DIR/RobotRaconteur_0.8"

# make sure we have cloned the full repo (including submodules)
git submodule init
git submodule update
# ------------------------
# Install Boost Libraries
# ------------------------
# requires gcc 4.8 ... We are just going to assume we have it
# download boost
wget "http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz"
# untar
tar -xzf "boost_1_60_0.tar.gz"
# remove the tarball
rm "boost_1_60_0.tar.gz"

# Install Boost
BOOST_DIR="$RR_DIR/boost_1_60_0"
cd $BOOST_DIR
./bootstrap
./b2 --with-date time --with-thread --with-system --with-regex --with-filesystem --with-chrono --with-atomic \
--layout=versioned variant=debug,release cflags=-fPIC cxxflags=-fPIC linkflags=-fPIC


# ------------------------
# Make the source code
# ------------------------
#cd $BASE_DIR
#mkdir build
#cd build
#cmake ..
#make 
#sudo make install
#sudo ldconfig

