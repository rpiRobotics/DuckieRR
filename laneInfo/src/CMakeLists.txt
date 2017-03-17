cmake_minimum_required(VERSION 2.8)
project(laneInfoPy)

configure_file(LineDetectorNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/LineDetectorNode.py)
configure_file(LaneInfoNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/LaneInfoNode.py)
configure_file(GroundProjectionNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/GroundProjectionNode.py)

configure_file(LaunchLaneInfo.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/LaunchLaneInfo.py)
