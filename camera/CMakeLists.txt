cmake_minimum_required(VERSION 2.8)
project(camera)

SET(CAMERA_ROBDEF ${ROBDEF_DIR}/camera.robdef)
add_subdirectory(CPP)
add_subdirectory(python)
