cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED DUAL_END_IMAGE_KEY)
    set(DUAL_END_IMAGE_KEY "app")
endif()
set(DUAL_END_PUBLIC_VERSION "0.5.20")
if(NOT DUAL_END_PUBLIC_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "Public firmware version must be numeric major.minor.patch")
endif()
# Keep the app image-key prefix required by already installed OTA validators.
set(PROJECT_VER "${DUAL_END_IMAGE_KEY}-${DUAL_END_PUBLIC_VERSION}")

include($ENV{IDF_PATH}/tools/cmake/version.cmake)
if(NOT "$ENV{IDF_VERSION}" VERSION_EQUAL "5.5.6")
    message(FATAL_ERROR "This dual-microphone firmware requires ESP-IDF 5.5.6")
endif()

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cytoscepter_dongle)

