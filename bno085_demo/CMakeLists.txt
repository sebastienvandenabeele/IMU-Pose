# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(bno085_example
  LANGUAGES C CXX ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(bno085_example
  main.cpp
  ../sh2/sh2.c
  ../sh2/sh2_SensorValue.c
  ../sh2/sh2_util.c
  ../sh2/shtp.c
)

target_include_directories(bno085_example PRIVATE
  ../sh2
)

target_link_libraries(bno085_example
  pico_stdlib
  hardware_i2c
)

pico_enable_stdio_usb(bno085_example 1)
pico_enable_stdio_uart(bno085_example 0)

pico_add_extra_outputs(bno085_example)
