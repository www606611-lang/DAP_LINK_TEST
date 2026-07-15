cmake_minimum_required(VERSION 3.22)

project(mspm0_workspace C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(DAP_LINK_TEST)
add_subdirectory(DIANSAI_MOTOR_DRIVER_BOARD)
add_subdirectory(PB21_SPI_LCD_TEST)
add_subdirectory(CAR_CONTROL)
