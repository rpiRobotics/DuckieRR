#!/usr/bin/env bash
# INSTALLER for Duckiebot RR interface
set -e
set -x

BASE_DIR=$(pwd)

# ------------------------
# Update Git Submodules
# ------------------------
# make sure we have cloned the full repo (including submodules)
git submodule init
git submodule update

# ------------------------
# Install Boost Libraries
# ------------------------
# requires gcc 4.8 ... We are just going to assume we have it
BOOST_DIR="$BASE_DIR/boost_1_60_0"
if [ ! -d "$BOOST_DIR" ]; then
	# download boost
	wget "http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz"
	# untar
	tar -xzvf "boost_1_60_0.tar.gz"
	# remove the tarball
	rm "boost_1_60_0.tar.gz"	
fi

# Install Boost
cd $BOOST_DIR
sh "bootstrap.sh"
./b2 --with-date_time --with-thread --with-system --with-regex --with-filesystem --with-chrono --with-atomic \
--layout=versioned variant=debug,release cflags=-fPIC cxxflags=-fPIC linkflags=-fPIC

# ------------------------
# Install RR Libraries
# ------------------------
# This is for the duckiebot... so we are going to assume these aren't installed.
RR_DIR="$BASE_DIR/RobotRaconteur_0.8"
RR_CPP_DIR="$RR_DIR/RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18"

cd $RR_DIR
if [ ! -d "$RR_CPP_DIR" ]; then
	tar -xvf "RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18.tar"
fi

# Robot Raconteur may be installed, so we need to check
# if RR not installed version will be empty, otherwise it will have version number
if [ "$(python -c "import pkg_resources;print pkg_resources.get_distribution('RobotRaconteur').version")" != "0.8" ]; then
	cd /
	sudo tar -xvf "$RR_DIR/RobotRaconteur-0.8.1-beta-Python.linux-armhf-py2.7-2016-07-18.tar"
	sudo apt-get install python-{serial,numpy,opencv,pygame}
	sudo usermod -a -G dialout ubuntu
	sudo usermod -a -G video ubuntu	
fi

# ------------------------
# Make the source code
# ------------------------
cd $BASE_DIR
#mkdir build
#cd build
#cmake ..
#make 
#sudo make install
#sudo ldconfig

