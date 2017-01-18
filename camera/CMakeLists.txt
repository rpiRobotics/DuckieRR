cmake_minimum_required(VERSION 2.8)
project(camera)

SET(CAMERA_ROBDEF ${CMAKE_CURRENT_SOURCE_DIR}/camera_service.robdef)
add_subdirectory(CPP)
add_subdirectory(python)
