# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Disable ccache because Windows paths may contain non-ASCII characters.
set(CCACHE_ENABLE OFF CACHE BOOL "Disable ccache for non-ASCII workspace paths" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(AM36-DoorCam)

# GCC 13.2 requires -O1 for esp_lcd with a 32 KB instruction cache.
idf_component_get_property(esp_lcd_lib esp_lcd COMPONENT_LIB)
target_compile_options(${esp_lcd_lib} PRIVATE -O1)
