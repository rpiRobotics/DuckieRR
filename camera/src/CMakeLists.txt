cmake_minimum_required(VERSION 2.8)
project(camera)

find_package(raspicam REQUIRED)

add_executable (camera_interface camera_interface.cpp)  
target_link_libraries (camera_interface ${raspicam_LIBS})