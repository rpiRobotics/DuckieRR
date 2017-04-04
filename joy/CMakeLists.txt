cmake_minimum_required(VERSION 2.8)
project(joy)

SET(JOYSTICK_ROBDEF ${ROBDEF_DIR}/joystick.robdef)
SET(JOYMAPPER_ROBDEF ${ROBDEF_DIR}/joymapper.robdef)

SET(DEFAULT_MAPPER_PARAMS ${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml)

# configure the file... need to know where the robdef file is
configure_file(JoyNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/JoyNode.py)
configure_file(JoyMapperNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/JoyMapperNode.py)