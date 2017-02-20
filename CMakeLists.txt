cmake_minimum_required(VERSION 2.8)
project(DuckieRR)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")

SET(Dependency_DIR "${CMAKE_SOURCE_DIR}/dependencies")

SET (RobotRaconteur_DIR "${Dependency_DIR}/RobotRaconteur_0.8/RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18")
SET (Boost_INCLUDE_DIR "${Dependency_DIR}/boost_1_60_0")
SET (BOOST_LIBRARY_DIR_DEBUG "${Dependency_DIR}/boost_1_60_0/stage")
SET (BOOST_LIBRARY_DIR_RELEASE "${Dependency_DIR}/boost_1_60_0/stage")

SET (RR_COMMON_CMAKE ${Dependency_DIR}/RobotRaconteurCommon.cmake)
SET (ROBDEF_DIR ${Dependency_DIR}/DuckiebotRobdef)
SET (DUCKIEBOT_ROBDEF ${ROBDEF_DIR}/duckiebot.robdef)


add_subdirectory(utils)
add_subdirectory(drive)
add_subdirectory(camera)
add_subdirectory(laneInfo)
