cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED DUAL_END_IMAGE_KEY)
    set(DUAL_END_IMAGE_KEY "app")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../dual_mic_version.cmake")
# Keep the app image-key prefix required by already installed OTA validators.
set(PROJECT_VER "${DUAL_END_IMAGE_KEY}-${DUAL_END_PUBLIC_VERSION}")

include($ENV{IDF_PATH}/tools/cmake/version.cmake)
# Keep the paired images on one SDK. SDK changes also require validation of
# the 20 ms audio budget and recovery timing on hardware.
set(DUAL_END_IDF_VERSION "6.1.1" CACHE STRING
    "ESP-IDF version selected for the paired dual-microphone firmware")
# ESP-IDF 6.1 export scripts no longer set IDF_VERSION in the environment;
# version.cmake above provides the components, so build the string from them.
set(DUAL_END_IDF_DETECTED
    "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
if(NOT DUAL_END_IDF_DETECTED VERSION_EQUAL "${DUAL_END_IDF_VERSION}")
    message(FATAL_ERROR
        "This dual-microphone firmware requires ESP-IDF ${DUAL_END_IDF_VERSION}; found '${DUAL_END_IDF_DETECTED}' (IDF_PATH=$ENV{IDF_PATH}). Override with -DDUAL_END_IDF_VERSION=<version> only after re-validating the audio path.")
endif()

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cytoscepter_scepter)

