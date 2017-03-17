cmake_minimum_required(VERSION 2.8)
project(drive)

# configure the file... need to know where the robdef file is
configure_file(DriveNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/DriveNode.py)