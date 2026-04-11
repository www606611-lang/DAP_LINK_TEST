set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TIARMCLANG_DIR "D:/TI/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS" CACHE PATH "Path to the TI Arm Clang toolchain")

set(CMAKE_C_COMPILER "${TIARMCLANG_DIR}/bin/tiarmclang.exe")
set(CMAKE_ASM_COMPILER "${TIARMCLANG_DIR}/bin/tiarmclang.exe")
set(CMAKE_AR "${TIARMCLANG_DIR}/bin/tiarmar.exe")
set(CMAKE_RANLIB "${TIARMCLANG_DIR}/bin/tiarmranlib.exe")

set(TIARMCLANG_COMMON_FLAGS "--target=arm-ti-none-eabi -mcpu=cortex-m0plus -march=thumbv6m -mfloat-abi=soft -mthumb")

set(CMAKE_C_FLAGS_INIT "${TIARMCLANG_COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${TIARMCLANG_COMMON_FLAGS}")

set(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -g")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O2")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Oz")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${TIARMCLANG_COMMON_FLAGS}")
