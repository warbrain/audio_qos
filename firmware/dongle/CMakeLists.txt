cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED DUAL_END_IMAGE_KEY)
    set(DUAL_END_IMAGE_KEY "app")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../dual_mic_version.cmake")
# Keep the app image-key prefix required by already installed OTA validators.
set(PROJECT_VER "${DUAL_END_IMAGE_KEY}-${DUAL_END_PUBLIC_VERSION}")

include($ENV{IDF_PATH}/tools/cmake/version.cmake)
# One verified toolchain for the paired images. Override only after re-validating
# the 20 ms audio budget and the recovery timing on hardware.
set(DUAL_END_IDF_VERSION "6.0.3" CACHE STRING
    "ESP-IDF version this dual-microphone firmware is verified against")
if(NOT "$ENV{IDF_VERSION}" VERSION_EQUAL "${DUAL_END_IDF_VERSION}")
    message(FATAL_ERROR
        "This dual-microphone firmware requires ESP-IDF ${DUAL_END_IDF_VERSION}; found '$ENV{IDF_VERSION}' (IDF_PATH=$ENV{IDF_PATH}). Override with -DDUAL_END_IDF_VERSION=<version> only after re-validating the audio path.")
endif()

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cytoscepter_dongle)

