cmake_minimum_required(VERSION 3.1)
project(ws281x_vendor)

add_library(rpi_ws281x STATIC "")

message(STATUS "Building for ${CMAKE_SYSTEM_PROCESSOR}")
if (${ROS_WS2811_MOCK_LIB})
    message(STATUS "Using mock rpi_ws281x library")
    target_sources(rpi_ws281x PRIVATE
            rpi_ws281x_mock/pwm.cpp
            rpi_ws281x_mock/rpihw.cpp
            rpi_ws281x_mock/ws2811.cpp
            )
    target_compile_features(rpi_ws281x PRIVATE
            cxx_generalized_initializers
            )
    target_include_directories(rpi_ws281x PUBLIC
            rpi_ws281x_mock
            )
else()
    message(STATUS "Using real rpi_ws281x library")
    target_sources(rpi_ws281x PRIVATE
            rpi_ws281x/dma.c
            rpi_ws281x/mailbox.c
            rpi_ws281x/pcm.c
            rpi_ws281x/pwm.c
            rpi_ws281x/rpihw.c
            rpi_ws281x/ws2811.c
            )
    target_include_directories(rpi_ws281x PUBLIC
            rpi_ws281x
            )
endif()
