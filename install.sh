#!/usr/bin/env bash
# INSTALLER for Duckiebot RR interface
set -e
set -x

# CHECK IF WE ARE ON THE PI
if [[ $(uname -m ) == "arm"* ]]; then
	echo "RASPBERRY PI DETECTED. RUNNING FULL INSTALL"
	ON_PI=true
else
	echo "THIS COMPUTER IS NOT A RASPERRY PI. SOME LIBRARIES WILL NOT BE INSTALLED."
	echo "... eventually I will also have the correct libraries listed for both..."
	ON_PI=false
fi


BASE_DIR=$(pwd)
DEPEND_DIR="$BASE_DIR/dependencies"

# ------------------------
# Any other packages...
# ------------------------
echo "CHECKING FOR NECESSARY PACKAGES..."
sudo apt-get install libyaml-cpp-dev -y
sudo apt-get install -y python-{serial,numpy,opencv,pygame}

# ------------------------
# Install Boost Libraries
# ------------------------
if [ "$ON_PI" = true ]; then 
	echo "INSTALLING BOOST..."
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
	echo "INSTALLING ROBOT RACONTEUR (VERSION 0.8)..."
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
		cd /
		sudo tar -xvf "$RR_DIR/RobotRaconteur-0.8.1-beta-Python.linux-armhf-py2.7-2016-07-18.tar"

		sudo usermod -a -G dialout ubuntu
		sudo usermod -a -G video ubuntu
	fi

	# ------------------------
	# Install OpenCV 3.2.0 
	# ------------------------
	echo "INSTALLING OPENCV (VERSION 3.2.0)..."
	OPENCV_DIR="$DEPEND_DIR/opencv-3.2.0"

	if [ ! -d "$OPENCV_DIR" ]; then
		cd $DEPEND_DIR
		# Download OpenCV
		wget "https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.2.0/opencv-3.2.0.zip"
		# unzip
		unzip "opencv-3.2.0.zip"
		# remove the zip file
		rm "opencv-3.2.0.zip"

	fi

	if [ ! -d "$OPENCV_DIR/build" ]; then
		cd $OPENCV_DIR
		mkdir -p build
		cd build 
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON2_EXECUTABLE=/usr/bin/python -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_INCLUDE_DIR2=/usr/include/arm-linux-gnueabihf/python2.7 -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython2.7.so -DPYTHON2_NUMPY_INCLUDE_DIRS=/usr/lib/python2.7/dist-packages/numpy/core/include/ ..
		make -j7 
		sudo make install
	fi

	# ------------------------
	# Extract and Make raspicam code
	# ------------------------
	echo "INSTALLING RASPICAM LIBRARY..."
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

fi #ON_PI

# -----------------------------------
# Make the Robdef Files and Libraries
# -----------------------------------
echo "BUILDING CPP THUNK SOURCE FILES..."
ROBDEF_DIR="$DEPEND_DIR/DuckiebotRobdef"
cd $ROBDEF_DIR
mkdir -p build
cd build
cmake ..
make -j7
sudo make install

# ------------------------
# Make the source code
# ------------------------
echo "BUILDING SOURCE CODE"
cd $BASE_DIR
mkdir -p build
cd build
cmake ..
make -j7
sudo make install
