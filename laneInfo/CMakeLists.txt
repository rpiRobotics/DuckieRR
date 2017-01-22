cmake_minimum_required(VERSION 2.8)
project(laneInfo)

set (DEFAULT_LANE_PARAMS ${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml)
add_subdirectory(include)
add_subdirectory(src)