cmake_minimum_required(VERSION 2.8)
project(laneInfo)

find_program(PYTHON "python")

if (PYTHON)
    set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
    set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
    set(DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/line_detector/__init__.py")
    set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/timestamp")

    add_custom_command(OUTPUT ${OUTPUT}
                       COMMMAND ${PYTHON} ${SETUP_PY} build
                       COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
                       DEPENDS ${DEPS})

    configure_file(${SETUP_PY_IN} ${SETUP_PY})

    add_custom_target(laneInfo ALL DEPENDS ${OUTPUT})

    install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
endif()