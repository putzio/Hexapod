#specify a min ver
cmake_minimum_required(VERSION 3.19)

#INCLUDE THE CMAKE IMPORT FILE
include(pico_sdk_import.cmake)

project(orchestra C CXX)

#inintialise the sdk
pico_sdk_init()

#Creating an executable target "orchestra" from a single source orchestra.c
add_executable(orchestra orchestra.cpp)
#libraries
target_link_libraries(orchestra pico_stdlib)
target_link_libraries(orchestra hardware_pwm)
target_link_libraries(orchestra hardware_clocks)
target_link_libraries(orchestra hardware_uart)
target_link_libraries(orchestra pico_double)
target_link_libraries(orchestra hardware_irq)#include "hardware/irq.h"
target_link_libraries(orchestra hardware_adc)
target_link_libraries(orchestra hardware_gpio)

#Generates different file formats
pico_add_extra_outputs(orchestra)