cmake_minimum_required(VERSION 2.8)
project(camera)

# configure the file... need to know where the robdef file is
configure_file(camera_interface.py.in camera_interface.py)