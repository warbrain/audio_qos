# USB and stereo buffer regression tests

Run `python -m pytest -q dongle/components/dongle_usb_device/tests` from the
firmware root. A native C11 compiler is required (MSVC Build Tools plus Windows
SDK on Windows, or GCC/Clang); tests fail rather than silently skip without it.

`bridge_test.c` includes the production bridge. It checks stereo startup and
fades, stretch/shrink corrections without channel swapping, overflow alignment,
underflow recovery, tick-zero and no-host timeouts, explicit sender session
reset, duplicate filtering and sequence wrap, gap concealment, and byte tails.

`test_uac_mic_stream.py` extracts production driver state and functions verbatim
and compiles them with `uac_mic_stream_test.c`. The simulated FIFO checks partial
writes, reset/reopen during a capture, pending/hardware drain, small FIFO flow
control, callback errors, and the one-millisecond endpoint input path.

The host compiler helper, UAC harness, and time/RTOS stubs were adapted from
the user-approved local `newpcb/firmware-0.4.1` reference. No reference source
directory is needed to run these tests. All generated files go into pytest's
temporary directory.
