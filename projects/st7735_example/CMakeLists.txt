cmake_minimum_required(VERSION 3.1)
project(sandbox C ASM)

set (ROOT "${PROJECT_SOURCE_DIR}/../..")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${ROOT}/cmake")

include(SetupToolchain NO_POLICY_SCOPE)
include(SetBuildFlags NO_POLICY_SCOPE)
include(functions NO_POLICY_SCOPE)

add_stm32_program(flash src/main.c)

add_subdirectory(${ROOT}/lib/drivers/st7735 "./st7735")
target_link_libraries(flash.elf PUBLIC st7735)
