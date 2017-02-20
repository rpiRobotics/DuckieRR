cmake_minimum_required(VERSION 2.8)
project(laneInfo)

set (DEFAULT_LANE_PARAMS ${CMAKE_CURRENT_SRC_DIR}/config/default_Lane.yaml)
set (DEFAULT_LD_PARAMS ${CMAKE_CURRENT_SRC__DIR}/config/default_LD.yaml)
set (DEFAULT_GP_PARAMS ${CMAKE_CURRENT_SRC_DIR}/config/default_GP.yaml)
set (DEFAULT_CAMEXT $(CMAKE_CURRENT_SRC_DIR)/config/default_CameraExtrinsic.yaml)
set (DEFAULT_CAMINT $(CMAKE_CURRENT_SRC_DIR)/config/default_CameraIntrinsic.yaml)

set (LINEDETECTOR_ROBDEF ${ROBDEF_DIR}/lineDetector.robdef)
set (LANEINFO_ROBDEF ${ROBDEF_DIR}/laneInfo.robdef)
set (GROUNDPROJECTION_ROBDEF ${ROBDEF_DIR}/groundProjection.robdef)

add_subdirectory(include)
add_subdirectory(src)
#add_subdirectory(CPP)