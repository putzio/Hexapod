#specify a min ver
cmake_minimum_required(VERSION 3.19)

#INCLUDE THE CMAKE IMPORT FILE
# include(pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include_directories(
    /workspaces/firmareworkspace/Hexapod/inc
    /workspaces/firmareworkspace/Hexapod/src
)
project(Hexapod C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#inintialise the sdk
pico_sdk_init()

#Creating an executable target "Hexapod" from a single source hexapod.cpp
add_executable(${PROJECT_NAME} src/hexapod.cpp)
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