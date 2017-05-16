cmake_minimum_required(VERSION 2.8)
project(laneControlRT)

# Load Boost / RobotRaconteur
include(${RR_COMMON_CMAKE})
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})

# Find the Robdef Libraries
find_package(DuckiebotRobdef CONFIG REQUIRED)
set(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} Duckiebot Duckiebot__Drive Duckiebot__LaneInfo)

# Add the real time library
set(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} rt)

# The executable and link libraries
SET (CMAKE_CXX_FLAGS "-std=c++11")
add_executable (laneControl-RT laneControl-RT.cpp)

target_link_libraries (laneControl-RT ${REQUIRED_LIBRARIES})