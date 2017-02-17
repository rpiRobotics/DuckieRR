#!/usr/bin/env bash
# INSTALLER for Duckiebot RR interface
set -e
set -x

BASE_DIR=$(pwd)
DEPEND_DIR="$BASE_DIR/dependencies"
# ------------------------
# Install Boost Libraries
# ------------------------
# requires gcc 4.8 ... We are just going to assume we have it
BOOST_DIR="$DEPEND_DIR/boost_1_60_0"
if [ ! -d "$BOOST_DIR" ]; then
	cd $DEPEND_DIR
	# download boost
	wget "http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz"
	# untar
	tar -xzvf "boost_1_60_0.tar.gz"
	# remove the tarball
	rm "boost_1_60_0.tar.gz"	
fi

# Install Boost
if [ ! -d "$BOOST_DIR/stage" ]; then
	cd $BOOST_DIR
	sh "bootstrap.sh"
	./b2 --with-date_time --with-thread --with-system --with-regex --with-filesystem --with-chrono --with-atomic \
	--layout=versioned variant=debug,release cflags=-fPIC cxxflags=-fPIC linkflags=-fPIC
fi

# ------------------------
# Install RR Libraries
# ------------------------
# This is for the duckiebot... so we are going to assume these aren't installed.
RR_DIR="$DEPEND_DIR/RobotRaconteur_0.8"
RR_CPP_DIR="$RR_DIR/RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18"

if [ ! -d "$RR_CPP_DIR" ]; then
	cd $RR_DIR
	tar -xvf "RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18.tar"
fi

# Robot Raconteur may be installed, so we need to check
# if RR not installed version will be empty, otherwise it will have version number
if [[ "$(python -c "import pkg_resources;print pkg_resources.get_distribution('RobotRaconteur').version")" != "0.8"* ]]; then
	if [[ $(uname -m) == "arm"* ]]
		cd /
		sudo tar -xvf "$RR_DIR/RobotRaconteur-0.8.1-beta-Python.linux-armhf-py2.7-2016-07-18.tar"
	fi
	sudo apt-get install -y python-{serial,numpy,opencv,pygame}
	
	if [[ $(uname -m) == "arm"* ]]
		sudo usermod -a -G dialout ubuntu
		sudo usermod -a -G video ubuntu
	fi	
fi

# ------------------------
# Extract and Make raspicam code
# ------------------------
RAPSICAM_DIR="$DEPEND_DIR/raspicam-0.1.3"
if [[ ! -d "$RASPICAM_DIR" && -x "$RASPICAM_DIR" ]]; then
	cd $DEPEND_DIR

	# install unzip since I guess it isn't installed
	sudo apt-get install unzip

	# unzip the zip file
	unzip "raspicam-0.1.3.zip"

	cd $RAPSICAM_DIR
	mkdir -p build
	cd build 
	cmake ..
	make
	sudo make install
	sudo ldconfig
fi
# -----------------------------------
# Make the Robdef Files and Libraries
# -----------------------------------
ROBDEF_DIR = "$DEPEND_DIR/DuckiebotRobdef"
cd $ROBDEF_DIR
mkdir -p build
cd build
cmake ..
sudo make install

# ------------------------
# Any other packages...
# ------------------------
sudo apt-get install libyaml-cpp-dev -y

# ------------------------
# Make the source code
# ------------------------
cd $BASE_DIR
mkdir -p build
cd build
cmake ..
sudo make install
