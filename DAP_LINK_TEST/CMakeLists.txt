cmake_minimum_required(VERSION 3.22)

project(dap_link_test C)

set(MSPM0_SDK_DIR "C:/TI/mspm0_sdk_2_10_00_04" CACHE PATH "Path to the MSPM0 SDK")
set(SYSCONFIG_CLI "C:/TI/sysconfig_1.26.2/sysconfig_cli.bat" CACHE FILEPATH "Path to sysconfig_cli.bat")
set(TIARMCLANG_DIR "D:/TI/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS" CACHE PATH "Path to the TI Arm Clang toolchain")
set(ARM_GCC_DIR "D:/google_download/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi" CACHE PATH "Path to the GNU Arm Embedded toolchain")

set(APP_NAME dap_link_test)
set(SYSCFG_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/cmsis_dsp_empty.syscfg")
set(SYSCFG_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/syscfg")
set(SYSCFG_PRODUCT "${MSPM0_SDK_DIR}/.metadata/product.json")

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  set(SYSCFG_COMPILER gcc)
  set(SYSCFG_LINKER "${SYSCFG_OUT_DIR}/device_linker.lds")
  set(SYSCFG_GENLIBS "${SYSCFG_OUT_DIR}/device.lds.genlibs")
  set(STARTUP_FILE "${MSPM0_SDK_DIR}/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c")
  set(APP_SUFFIX ".elf")
elseif(CMAKE_C_COMPILER_ID STREQUAL "TIClang")
  set(SYSCFG_COMPILER ticlang)
  set(SYSCFG_LINKER "${SYSCFG_OUT_DIR}/device_linker.cmd")
  set(SYSCFG_GENLIBS "${SYSCFG_OUT_DIR}/device.cmd.genlibs")
  set(STARTUP_FILE "${MSPM0_SDK_DIR}/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c")
  set(APP_SUFFIX ".out")
else()
  message(FATAL_ERROR "Unsupported compiler: ${CMAKE_C_COMPILER_ID}")
endif()

set(SYSCFG_C "${SYSCFG_OUT_DIR}/ti_msp_dl_config.c")
set(SYSCFG_H "${SYSCFG_OUT_DIR}/ti_msp_dl_config.h")
set(SYSCFG_OPT "${SYSCFG_OUT_DIR}/device.opt")

set(APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/app/lcd_status.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/app/uart_display.c"
)

set(COMMON_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/common/delay.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/common/timer.c"
)

set(MODULE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/encoder.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/icm20948.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/motor.c"
  # ST7789 复用这里的字库；OLED 驱动本体 modules/oled.c 不参与编译。
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/OLED_Data.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/st7789.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules/uart0_dma.c"
)

add_custom_command(
  OUTPUT
    "${SYSCFG_C}"
    "${SYSCFG_H}"
    "${SYSCFG_OPT}"
    "${SYSCFG_LINKER}"
    "${SYSCFG_GENLIBS}"
  COMMAND "${SYSCONFIG_CLI}"
          --compiler
          "${SYSCFG_COMPILER}"
          --product
          "${SYSCFG_PRODUCT}"
          --output
          "${SYSCFG_OUT_DIR}"
          "${SYSCFG_INPUT}"
  DEPENDS "${SYSCFG_INPUT}"
  COMMENT "Generating SysConfig files for ${SYSCFG_COMPILER}"
  VERBATIM
)

add_custom_target(syscfg
  DEPENDS
    "${SYSCFG_C}"
    "${SYSCFG_H}"
    "${SYSCFG_OPT}"
    "${SYSCFG_LINKER}"
    "${SYSCFG_GENLIBS}"
)

add_executable(${APP_NAME}
  "${CMAKE_CURRENT_SOURCE_DIR}/cmsis_dsp_empty.c"
  "${STARTUP_FILE}"
  "${SYSCFG_C}"
  ${MODULE_SOURCES}
  ${COMMON_SOURCES}
  ${APP_SOURCES}
)

add_dependencies(${APP_NAME} syscfg)

set_target_properties(${APP_NAME} PROPERTIES SUFFIX "${APP_SUFFIX}")

target_compile_features(${APP_NAME} PRIVATE c_std_99)

target_compile_definitions(${APP_NAME} PRIVATE ARM_MATH_CM0)

target_include_directories(${APP_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/app"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/common"
  "${SYSCFG_OUT_DIR}"
  "${MSPM0_SDK_DIR}/source"
  "${MSPM0_SDK_DIR}/source/third_party/CMSIS/Core/Include"
  "${MSPM0_SDK_DIR}/source/third_party/CMSIS/DSP/Include"
)

target_compile_options(${APP_NAME} PRIVATE
  "@${SYSCFG_OPT}"
  -Wall
)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_include_directories(${APP_NAME} PRIVATE
    "${ARM_GCC_DIR}/arm-none-eabi/include/newlib-nano"
    "${ARM_GCC_DIR}/arm-none-eabi/include"
  )

  target_compile_options(${APP_NAME} PRIVATE
    -ffunction-sections
    -fdata-sections
    -gstrict-dwarf
  )

  target_link_directories(${APP_NAME} PRIVATE
    "${MSPM0_SDK_DIR}/source"
    "${ARM_GCC_DIR}/arm-none-eabi/lib/thumb/v6-m/nofp"
  )

  target_link_options(${APP_NAME} PRIVATE
    -nostartfiles
    "-T${SYSCFG_GENLIBS}"
    "-Wl,-T,${SYSCFG_LINKER}"
    "-Wl,-Map,${APP_NAME}.map"
    -static
    "-Wl,--gc-sections"
    --specs=nano.specs
    --specs=nosys.specs
  )

  target_link_libraries(${APP_NAME} PRIVATE
    gcc
    c
    m
  )
elseif(CMAKE_C_COMPILER_ID STREQUAL "TIClang")
  target_compile_options(${APP_NAME} PRIVATE
    -gdwarf-3
  )

  target_link_directories(${APP_NAME} PRIVATE
    "${MSPM0_SDK_DIR}/source"
    "${SYSCFG_OUT_DIR}"
    "${TIARMCLANG_DIR}/lib"
  )

  target_link_options(${APP_NAME} PRIVATE
    -Wl,-u,_c_int00
    "${SYSCFG_LINKER}"
    "-Wl,-m,${APP_NAME}.map"
    -Wl,--rom_model
    -Wl,--warn_sections
  )

  target_link_libraries(${APP_NAME} PRIVATE
    device.cmd.genlibs
    "${TIARMCLANG_DIR}/lib/libc.a"
  )
endif()
