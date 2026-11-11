# Firmware

ESP-IDF firmware source for the Scepter and Dongle roles. The two projects share
protocol, codec, recovery, and runtime components in the components directory.

## Build

Use ESP-IDF v5.5.6 and build each role from its own directory:

    cd scepter
    idf.py build

    cd ..\dongle
    idf.py build

Generated outputs stay in local build directories and are not part of this source tree.
