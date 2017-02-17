cmake_minimum_required(VERSION 2.8)
project(CameraNode)

# Load Raspicam 
find_package(raspicam REQUIRED)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${raspicam_LIBS})

# Load Boost / RobotRaconteur
include(${RR_COMMON_CMAKE})
SET(REQUIRED_LIBRARIES, ${REQUIRED_LIBRARIES} ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})

# Find the Robdef Libraries
find_package(DuckiebotRobdef CONFIG REQUIRED)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} Duckiebot Duckiebot__Camera)

# The executable and link libraries
SET (CMAKE_CXX_FLAGS "-std=c++11")
add_executable (CameraNode main.cpp CameraNode.cpp)

target_link_libraries (CameraNode ${REQUIRED_LIBRARIES})
