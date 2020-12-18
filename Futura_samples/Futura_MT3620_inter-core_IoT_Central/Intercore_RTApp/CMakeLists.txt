cmake_minimum_required(VERSION 3.10)
PROJECT(Futura_MT3620_inter-core_IoT_Central_RT)
azsphere_configure_tools(TOOLS_REVISION "20.07")
add_executable(${PROJECT_NAME} main.c logical-intercore.c logical-dpc.c mt3620-intercore.c mt3620-uart-poll.c mt3620-timer.c)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/linker.ld)
azsphere_target_add_image_package(${PROJECT_NAME})

