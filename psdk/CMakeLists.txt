cmake_minimum_required(VERSION 3.16)
# Include from the device's existing PSDK application; no PSDK license/credentials bundled.
if(NOT DEFINED DJI_PSDK_ROOT)
  message(FATAL_ERROR "Set DJI_PSDK_ROOT to the existing DJI SDK checkout")
endif()
add_library(m300_voice_runtime STATIC m300_runtime.c command_guard.c host_integration.c)
target_compile_features(m300_voice_runtime PRIVATE c_std_11)
target_include_directories(m300_voice_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DJI_PSDK_ROOT}/psdk_lib/include PRIVATE ${DJI_PSDK_ROOT}/samples/sample_c/module_sample/utils)
target_link_libraries(m300_voice_runtime PUBLIC pthread m)
# cJSON.c and the platform-specific libpayloadsdk.a are already linked by the PSDK host sample.
