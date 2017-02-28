cmake_minimum_required(VERSION 2.8)
project(DuckiebotRobdef)

# List all of the target libraries to be created
# (Namespace1.Namespace2 --> Namespace1__Namespace2)
SET (ALLTARGETS 
        Duckiebot 
        Duckiebot__Camera 
        Duckiebot__GroundProjection 
        Duckiebot__LaneInfo 
        Duckiebot__LineDetector
        Duckiebot__Drive)

# Find all of the Robdef Files
FILE(GLOB ROBDEFS "${CMAKE_CURRENT_SOURCE_DIR}/*.robdef")

# Make a directory for the generated thunksource
FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/CPP_Thunk)
SET(THUNK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CPP_Thunk)

# Gen Lists all of the expected Thunksource files
foreach(LIB_NAME ${ALLTARGETS})
    SET (${LIB_NAME}HEADERS 
            ${THUNK_DIR}/${LIB_NAME}.h
            ${THUNK_DIR}/${LIB_NAME}_stubskel.h)
    SET (${LIB_NAME}SOURCE
        ${THUNK_DIR}/${LIB_NAME}_stubskel.cpp)
    SET (${LIB_NAME}FILES
            ${${LIB_NAME}HEADERS} 
            ${${LIB_NAME}SOURCE} )
    SET (THUNK_FILES ${THUNK_FILES} ${${LIB_NAME}FILES})
    SET (THUNK_HEADERS ${THUNK_HEADERS} ${${LIB_NAME}HEADERS})
endforeach(LIB_NAME)

# Set up some relative dependencies so we can find RR and Boost
SET(Dependency_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
SET (RobotRaconteur_DIR "${Dependency_DIR}/RobotRaconteur_0.8/RobotRaconteur-0.8.1-beta-CPP-SDK-gcc-linux-armhf-2016-07-18")
SET (Boost_INCLUDE_DIR "${Dependency_DIR}/boost_1_60_0")
SET (BOOST_LIBRARY_DIR_DEBUG "${Dependency_DIR}/boost_1_60_0/stage")
SET (BOOST_LIBRARY_DIR_RELEASE "${Dependency_DIR}/boost_1_60_0/stage")

# Load Boost / RobotRaconteur
include(${Dependency_DIR}/RobotRaconteurCommon.cmake)

# Add Boost and Robot Raconteur as Required Libraries
SET (REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})

# Run RobotRaconteurGen to get all of the thunksource files
add_custom_command(OUTPUT ${THUNK_FILES}
    COMMAND ${RobotRaconteur_GEN} --thunksource --lang=cpp ${ROBDEFS}
    DEPENDS ${ROBDEFS}
    WORKING_DIRECTORY ${THUNK_DIR}
    COMMENT "Generating Thunk Source Files")

# Create a library for each target 
foreach(LIB_NAME ${ALLTARGETS})
    add_library(${LIB_NAME} ${${LIB_NAME}FILES})
    target_link_libraries(${LIB_NAME} ${REQUIRED_LIBRARIES})
    target_include_directories(${LIB_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${THUNK_DIR}>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
endforeach(LIB_NAME)

# Install all header files
install(FILES ${THUNK_HEADERS} DESTINATION include/${PROJECT_NAME})

# install all libraries
install(TARGETS ${ALLTARGETS}
    EXPORT ${PROJECT_NAME}-targets
    DESTINATION lib/${PROJECT_NAME})

# install the config file so that we can find these libraries later
install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})