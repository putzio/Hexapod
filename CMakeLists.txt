#specify a min ver
cmake_minimum_required(VERSION 3.19)

#INCLUDE THE CMAKE IMPORT FILE
include(pico_sdk_import.cmake)

project(Hexapod C CXX)

#inintialise the sdk
pico_sdk_init()

#Creating an executable target "Hexapod" from a single source hexapod.cpp
add_executable(${PROJECT_NAME} hexapod.cpp)
#libraries
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pwm
    hardware_clocks
    hardware_uart
    pico_double
    hardware_irq
    hardware_adc
    hardware_gpio
    ) 

#Generates different file formats
pico_add_extra_outputs(Hexapod)