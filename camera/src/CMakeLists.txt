cmake_minimum_required(VERSION 2.8)
project(camera)

# Load Raspicam 
find_package(raspicam REQUIRED)

# Load Boost / RobotRaconteur
SET (Boost_ADDITIONAL_VERSIONS 1.60.0 1.60)
SET (Boost_USE_STATIC_LIBS ON)
SET (Boost_USE_MULTITHREADED ON)
SET (Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread REQUIRED)
add_definitions(-DBOOST_ALL_NO_LIB)

if (CMAKE_COMPILER_IS_GNUCXX)
    find_package(OpenSSL)
    set (RobotRaconteur_EXTRA_LIBRARIES ${RobotRaconteur_EXTRA_LIBRARIES} ${OPENSSL_LIBRARIES} pthread rt z)
endif()

if (CMAKE_GENERATOR STREQUAL Xcode)
    set (OPENSSL_USE_STATIC_LIBS TRUE)
    find_package(OpenSSL)

    include(CMakeFindFrameworks)
    set (CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvm.clang.1_0")
    set (CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    CMAKE_FIND_FRAMEWORKS(CoreFoundation)
    CMAKE_FIND_FRAMEWORKS(Security)
    include_directories(${CoreFoundation_FRAMEWORKS}/Headers ${Security_FRAMEWORKS}/Headers)
endif()

include_directories(${Boost_INCLUDE_DIRS})

find_package(RobotRaconteur REQUIRED)

# The executable and link libraries
#add_executable (camera_interface camera_service_stubskel.cpp camera_interface.cpp)
add_executable (camera_interface camera_interface.cpp)

target_link_libraries (camera_interface ${raspicam_LIBS} ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})
