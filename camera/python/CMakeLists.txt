cmake_minimum_required(VERSION 2.8)
project(camera)

# configure the file... need to know where the robdef file is
configure_file(CameraNode.in.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/CameraNode.py)