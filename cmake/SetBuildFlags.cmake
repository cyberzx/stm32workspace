list(APPEND DEFINES -DSTM32)
list(APPEND DEFINES -DSTM32F1)
list(APPEND DEFINES -DSTM32F103C8Tx)
list(APPEND DEFINES -DSTM32F10X_MD)
list(APPEND DEFINES -DUSE_STDPERIPH_DRIVER)
#list(APPEND DEFINES -DDEBUG)
list(JOIN DEFINES " " DEFINES)

list(APPEND CFLAGS -mcpu=cortex-m3)
list(APPEND CFLAGS -mthumb)
list(APPEND CFLAGS -mfloat-abi=soft)
list(APPEND CFLAGS -O3)
list(APPEND CFLAGS -g3)
list(APPEND CFLAGS -Wall)
list(APPEND CFLAGS -ffunction-sections)
list(APPEND CFLAGS -fdata-sections)
list(JOIN CFLAGS " " CFLAGS)

set (CMAKE_C_FLAGS "${CFLAGS} ${DEFINES}")

# print memory mapping to output.map
list(APPEND LINKER_FLAGS -Wl,-Map=output.map)
# enable garbage collection for unused sections
list(APPEND LINKER_FLAGS -Wl,--gc-sections)
# add linker script
list(APPEND LINKER_FLAGS -T\"${ROOT}/scripts/LinkerScript.ld\")

list(APPEND LINKER_FLAGS -lm)

list(JOIN LINKER_FLAGS " " CMAKE_EXE_LINKER_FLAGS)
