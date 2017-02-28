cmake_minimum_required(VERSION 2.8)
project(DriveByAngleCPP)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")

SET(Dependency_DIR "${CMAKE_SOURCE_DIR}/../../../dependencies")

SET (RobotRaconteur_DIR "${Dependency_DIR}/RobotRaconteur_0.8/RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18")
SET (Boost_INCLUDE_DIR "${Dependency_DIR}/boost_1_60_0")
SET (BOOST_LIBRARY_DIR_DEBUG "${Dependency_DIR}/boost_1_60_0/stage")
SET (BOOST_LIBRARY_DIR_RELEASE "${Dependency_DIR}/boost_1_60_0/stage")

SET (RR_COMMON_CMAKE ${Dependency_DIR}/RobotRaconteurCommon.cmake)
SET (ROBDEF_DIR ${Dependency_DIR}/DuckiebotRobdef)
SET (DUCKIEBOT_ROBDEF ${ROBDEF_DIR}/duckiebot.robdef)


# Load OpenCV
find_package(OpenCV REQUIRED)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${OpenCV_LIBS})

# Load Boost / RobotRaconteur
include(${RR_COMMON_CMAKE})
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${RobotRaconteur_LIBRARY}  ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})

# Find the Thunk Source
find_package(DuckiebotRobdef CONFIG REQUIRED)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} Duckiebot Duckiebot__Camera Duckiebot__Drive)

# Find libyaml-cpp-dev
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
link_directories(${YAML_CPP_LIBRARY_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
SET (REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${YAML_CPP_LIBRARIES})

#configure_file(GroundProjectionNode.in.h GroundProjectionNode.h)
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

# The executable and link libraries
SET (CMAKE_CXX_FLAGS "-std=c++11")
add_executable (DriveByAngle main.cpp)

target_link_libraries (DriveByAngle ${REQUIRED_LIBRARIES})

