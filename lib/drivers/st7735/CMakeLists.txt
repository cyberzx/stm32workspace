add_library(st7735 st7735.c)
target_link_libraries(st7735 PUBLIC std_periph)
target_include_directories(st7735 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
