
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

#include(pico-sdk/pico_sdk_init.cmake)
#include(pico-sdk/pico_sdk_import.cmake)

#set(PICO_SDK_VERSION_REVISION 0)

project(pico_gg_hd)

pico_sdk_init()

add_executable(pico_gg_hd 
	${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/libdvi)

pico_generate_pio_header(pico_gg_hd ${CMAKE_CURRENT_LIST_DIR}/src/gg_capture.pio)
pico_generate_pio_header(pico_gg_hd ${CMAKE_CURRENT_LIST_DIR}/src/lcd_send.pio)

target_link_libraries(pico_gg_hd pico_stdlib pico_time 
hardware_pio hardware_sync hardware_dma hardware_clocks pico_multicore 
pico_time hardware_adc hardware_pwm hardware_interp libdvi)

pico_add_extra_outputs(pico_gg_hd)

pico_enable_stdio_usb(pico_gg_hd 1)
pico_enable_stdio_uart(pico_gg_hd 1)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif()