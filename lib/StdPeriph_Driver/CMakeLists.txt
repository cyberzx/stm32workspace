file(GLOB SRCLIST ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_library(std_periph "${SRCLIST}")
target_include_directories(std_periph INTERFACE
                              ${CMAKE_CURRENT_SOURCE_DIR}/inc
                              ${ROOT}/lib/CMSIS/core
                              ${ROOT}/lib/CMSIS/device
                          )
target_include_directories(std_periph PRIVATE
                              ${CMAKE_CURRENT_SOURCE_DIR}/inc
                              ${ROOT}/lib/CMSIS/core
                              ${ROOT}/lib/CMSIS/device
                          )
