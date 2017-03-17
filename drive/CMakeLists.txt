cmake_minimum_required(VERSION 2.8)
project(drive)

SET(DRIVE_ROBDEF ${ROBDEF_DIR}/drive.robdef)
add_subdirectory(include)
add_subdirectory(src)