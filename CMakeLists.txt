#specify a min ver
cmake_minimum_required(VERSION 3.19)

#INCLUDE THE CMAKE IMPORT FILE
include(pico_sdk_import.cmake)

project(orchestra C CXX)

#inintialise the sdk
pico_sdk_init()
#add my libraries
set (SRC
    lib/S.cpp 
    lib/S.h
)
add_library(${PROJECT_NAME}-lib STATIC ${SRC})

#Creating an executable target "orchestra" from a single source orchestra.c
add_executable(${PROJECT_NAME} orchestra.cpp)
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

target_link_libraries(${PROJECT_NAME}-lib 
    pico_stdlib
    hardware_pwm
    hardware_clocks
    pico_double
    )

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}-lib)
#Generates different file formats
pico_add_extra_outputs(orchestra)