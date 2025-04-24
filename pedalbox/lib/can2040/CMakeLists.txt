add_library(can2040
    can2040.h
    can2040.c
)

target_link_libraries(can2040
    hardware_resets
    cmsis_core
)

target_include_directories(can2040 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")