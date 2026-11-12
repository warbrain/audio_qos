# Firmware

ESP-IDF firmware source for the Scepter and Dongle roles. The two projects share
protocol, codec, recovery, and runtime components in the components directory.

## Paired release version

The current dual-microphone Scepter/Dongle pair is **0.5.26**. Both CMake
projects read the public version from `dual_mic_version.cmake`; each role's
`SCEPTER_FW_VERSION` macro and README must match it. The previous version drift
(Scepter 0.5.26, Dongle 0.5.21) came from separately maintained per-project
literals. OTA validates the target project and app/factory image key, while the
pairing protocol has its own version field. Future paired releases should
change the shared version once and update both role documents together.

## Build

Use ESP-IDF v5.5.6 and build each role from its own directory:

    cd scepter
    idf.py build

    cd ..\dongle
    idf.py build

Generated outputs stay in local build directories and are not part of this source tree.
