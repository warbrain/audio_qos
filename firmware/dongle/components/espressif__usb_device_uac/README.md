# Local UAC driver override

Based on Espressif `usb_device_uac` 1.3.1 (Apache-2.0, see `license.txt`), with
the USB microphone fixes from the locally supplied `firmware-0.4.1` source.
The project component name overrides `managed_components/espressif__usb_device_uac`;
refreshing managed dependencies therefore does not remove the fixes.

The driver preserves partial FIFO writes, separates producer storage from FIFO
capacity, rejects captures from retired sessions, and resets queued PCM at a
USB endpoint completion boundary. The dongle uses the optional one-millisecond
USB-boundary input path and initializes this component before enumeration.
The configured format remains 16 kHz, 16-bit, two physical microphone channels.

The matching TinyUSB audio driver is retained in
`../dongle_usb_device/tinyusb/audio_device.c`. Its component CMake replaces only
the managed TinyUSB audio source; the remaining TinyUSB sources stay managed.
The two local fixes add the pre-load session boundary hook and correct the
minimum FIFO size test for flow control.

Run the host regressions from the firmware root:

    python -m pytest -q dongle/components/dongle_usb_device/tests

These compile real production C with time/USB boundary stubs: 10 bridge
scenarios and 12 USB/FIFO/session scenarios, plus initialization-order checking.
They do not substitute for recording, USB reconnect, or wireless hardware tests.
