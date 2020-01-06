
function(add_stm32_program name sources)
  set(ELF_NAME ${name}.elf)
  set(BIN_NAME ${name}.bin)
  add_executable(${ELF_NAME} ${sources}
                 ${ROOT}/lib/startup/startup_stm32.s)

  add_custom_command(
    TARGET ${ELF_NAME} POST_BUILD
    COMMAND ${OBJ_COPY} -O binary "${ELF_NAME}" "${BIN_NAME}"
    COMMAND ${SHOW_SIZE} "${ELF_NAME}"
  )

  add_custom_target(flash COMMAND ${ROOT}/scripts/flash.sh "${CMAKE_CURRENT_BINARY_DIR}/${BIN_NAME}")

  add_subdirectory(${ROOT}/lib/StdPeriph_Driver "./std_periph")
  target_link_libraries(${ELF_NAME} PUBLIC std_periph)
endfunction()
