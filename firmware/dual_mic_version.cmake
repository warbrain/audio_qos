# One public version for the paired dual-microphone Scepter and Dongle images.
set(DUAL_END_PUBLIC_VERSION "0.5.26")
if(NOT DUAL_END_PUBLIC_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "Public firmware version must be numeric major.minor.patch")
endif()
