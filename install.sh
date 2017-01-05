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
BOOST_DIR="$BASE_DIR/boost_1_60_0"
if [! -d "$BOOST_DIR"]; then
	# download boost
	wget "http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz"
	# untar
	tar -xzvf "boost_1_60_0.tar.gz"
	# remove the tarball
	rm "boost_1_60_0.tar.gz"

	# Install Boost

	cd $BOOST_DIR
	./bootstrap
	./b2 --with-date time --with-thread --with-system --with-regex --with-filesystem --with-chrono --with-atomic \
	--layout=versioned variant=debug,release cflags=-fPIC cxxflags=-fPIC linkflags=-fPIC
fi

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

