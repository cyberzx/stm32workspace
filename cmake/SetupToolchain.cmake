cmake_policy(SET CMP0065 NEW) # do not add -rdynamic when linking

set (TOOLCHAIN_PATH $ENV{STM32_TOOLCHAIN_PATH})

set (CMAKE_SYSTEM_NAME stm32)
set (CMAKE_SYSTEM_PROCESSOR arm)
set (CMAKE_ENABLE_EXPORTS False)
set (CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc")
set (CMAKE_ASM_COMPILER "${TOOLCHAIN_PATH}/bin/arm-none-eabi-as")
set (OBJ_COPY "${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy")
set (SHOW_SIZE "${TOOLCHAIN_PATH}/bin/arm-none-eabi-size")

