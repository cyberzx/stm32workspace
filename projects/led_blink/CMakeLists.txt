cmake_minimum_required(VERSION 3.1)
project(led_blink C ASM)

set (ROOT "${PROJECT_SOURCE_DIR}/../..")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${ROOT}/cmake")

include(SetupToolchain NO_POLICY_SCOPE)
include(SetBuildFlags NO_POLICY_SCOPE)
include(functions NO_POLICY_SCOPE)

add_stm32_program(led_blink src/main.c)
