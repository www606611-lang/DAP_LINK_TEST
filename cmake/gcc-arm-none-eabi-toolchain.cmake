set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_GCC_DIR "D:/google_download/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi" CACHE PATH "Path to the GNU Arm Embedded toolchain")

set(CMAKE_C_COMPILER "${ARM_GCC_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${ARM_GCC_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_AR "${ARM_GCC_DIR}/bin/arm-none-eabi-ar.exe")
set(CMAKE_RANLIB "${ARM_GCC_DIR}/bin/arm-none-eabi-ranlib.exe")
set(CMAKE_OBJCOPY "${ARM_GCC_DIR}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${ARM_GCC_DIR}/bin/arm-none-eabi-size.exe")

set(GCC_COMMON_FLAGS "-mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft")

set(CMAKE_C_FLAGS_INIT "${GCC_COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${GCC_COMMON_FLAGS}")

set(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -g")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O2")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${GCC_COMMON_FLAGS}")
