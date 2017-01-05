cmake_minimum_required(VERSION 2.8)
project(adafruit_drivers)

find_program(PYTHON "python")

if (PYTHON)

    set(SETUP_PY_IN    "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
    set(SETUP_PY       "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
    set(DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/include/Adafruit_ADS1x15/Adafruit_ADS1x15.py" 
                    "${CMAKE_CURRENT_SOURCE_DIR}/include/Adafruit_GPIO/__init__.py"
                    "${CMAKE_CURRENT_SOURCE_DIR}/include/Adafruit_I2C/__init__.py" 
                    "${CMAKE_CURRENT_SOURCE_DIR}/include/Adafruit_MotorHAT/__init__.py"
                    "${CMAKE_CURRENT_SOURCE_DIR}/include/Adafruit_PWM_Servo_Driver/__init__.py")
    
    set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/timestamp")

    configure_file(${SETUP_PY_IN} ${SETUP_PY})

    add_custom_command(OUTPUT ${OUTPUT}
                       COMMMAND ${PYTHON} ${SETUP_PY} build
                       COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
                       DEPENDS ${DEPS})

    add_custom_target(adafruit_drivers ALL DEPENDS ${OUTPUT})

    install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
endif()