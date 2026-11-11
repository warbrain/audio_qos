# Audio / communication affinity and timing windows

ESP32-S3 dual-core builds use audio owner Core 1 and communication/USB Core 0. Single-core builds map both to Core 0. Task priorities and existing stacks are unchanged. Main remains Core 0, 240 MHz is the configured CPU speed, tick rate remains 1 kHz. Wi-Fi keeps its existing configured Core 0.

## Task ownership

- Scepter uses separate Core 1 owners: the priority-6 DMA consumer reads capture blocks, the priority-5 microphone task owns DSP/spatial state, and the priority-4 frame-dispatch worker invokes the callback and both independent G.722/Opus encoder process calls. L/R codecs stay together on that worker. FFT diagnostics and formatting run in background tasks; I2S DMA/interrupt installation remains on main Core 0.
- Scepter ESP-NOW TX, sender recovery, link status, short-control worker, USB Serial/JTAG pairing/Type-C OTA and wireless OTA workers are explicitly Core 0 at their previous priorities/stacks.
- Dongle ESP-NOW RX/recovery/decode owner (priority 6, 20480-byte stack) is Core 1. Both decoder processes and session decoder resets remain serialized there. Initial G.722 preparation remains in main before receive callbacks are registered.
- Active composite TinyUSB task is explicitly Core 0 through `tusb_cfg.task.xCoreID`, retaining priority 5 / 4096-byte stack. The independent UAC TinyUSB default is skipped and does not control this task.
- Dongle UAC `usb_mic_task` remains priority 7 / 4096 bytes and is explicitly Core 0. In the existing USB-boundary mode it only waits; actual 1 ms PCM reads occur in the TinyUSB IN callback. Boundaries, FIFO/session flushing, underrun and short-write counters are unchanged.
- Dongle feedback/control TX, idle heartbeat, link status, CDC command/response, HID service and deferred audio telemetry are Core 0. Unused native pairing/OTA task creation paths also use communication policy.
- Display, sensors, keys/wheel, motor, RGB and other background tasks retain their old affinity. Core 1 is not exclusively reserved.
- Encoder session resets remain serialized by the sender mutex. Codec/layout transitions also use the existing dispatch-control gate, outside the regular DSP path. Initial encoder creation is in main. The existing bounded PCM dispatch queue is unchanged by this runtime counter fix.

## Measurement scope

Stages are numeric: 0 `encode`, 1 `mic_pipeline`, 2 `decode`, 3 `rx_poll`. The encoder/decode sample covers the single call handling both microphones. Each elapsed time is wall time, including preemption, not CPU utilization or power consumption.

The legacy runtime stage name `mic_pipeline` now measures the asynchronous frame callback wall work, including sender admission, encoding and radio submission; the DSP owner records its immutable completion. Local DSP/spatial/bounded-handoff timings are published separately by `audio.pipeline` and exclude that asynchronous callback. Callback outcomes distinguish codec failure from admission that never attempted encoding. Actual transmit/USB delivery remains a separate transport observation.

`encode` is nested in the legacy runtime `mic_pipeline` callback stage, and `decode` is nested in `rx_poll`; do not add their totals. Encoder preparation/reset failures retain the original error/rollback behavior. Existing UART/health logs can still delay a task and appear as preemption/I/O in wall measurements.

A per-stage window rolls about every 5 seconds, and flushes on the next sample when codec/session changes. Partial windows are identified by timestamps; no-sample periods generate no fictional measurements. Counters are window deltas: samples are timed attempts, failures are failed attempts, successful is samples minus failures, sum includes failures. All-failed, empty or overflowed summaries are unknown. Over-budget is strictly greater than 20000 us. Invalid duration/core and saturated overflow counters are explicit.

The nine histogram buckets have finite upper bounds `[100,250,500,1000,2500,5000,10000,20000]` us and an unbounded ninth bucket. P95 is the nearest-rank bucket upper bound; UINT32_MAX means the unbounded bucket, not an exact duration.

## Deferred diagnostics and resource budget

Each runtime stage has exactly one producer; separate stages may use different tasks. Per-stage windows stay owner-local. Their shared nonzero window ID is allocated atomically, skipping zero at uint32 wrap, and their shared telemetry-drop counter uses atomic saturation. No audio-path lock or producer wait is introduced. Each owner only updates small counters and copies snapshots via `xQueueSend(..., 0)`. A separate diagnostic queue has capacity 4; a full queue increments telemetry_drop without blocking or dropping audio. The Core 0 logger has priority 2 and a 6144-byte stack. Boot JSON includes the actual item byte size; capacity times item size excludes FreeRTOS metadata. Optional telemetry initialization runs after mandatory services and the existing image confirmation. Its queue/worker must both exist before an atomic release/acquire enabled flag permits audio producers to publish; the logger also waits for that ready notification. Initialization failure reports profiling_disabled and continues original audio/control services without exposing a queue that can then be deleted.

Heap free/minimum and owner lifetime stack watermark are collected by the logger after dequeue, with `resource_at_us` and `resources_scope=logger_time`. Global heap includes retained inactive codec states and other device services; this is not an isolated codec heap measurement. No global trace or run-time CPU-stat feature is enabled.

## JSON wire/log schema

Actual device identity is repeated with every window so a monitor attached after boot can identify it. `audio.device_boot` schema 1 shares boot_id (8 hex digits) and role (0 Scepter / 1 Dongle), with parts:

0 device_id derived from the actual local STA MAC, or unknown on read failure; 1 actual chip cores/revision and compiled IDF target; 2 running app version; 3 configured cpu_mhz and audio_core/comm_core; 4 profiling enabled, queue_capacity, logger_stack, logger_priority; 5 queue_item_bytes, observed logger_core and resources_scope. USB 0101 does not prove chip or firmware identity.

`audio.codec_perf` schema 1 uses the same boot_id/role plus a device-wide, atomically allocated nonzero window_seq and numeric part 0..12. Sequence allocation is unique across concurrent stage producers until the uint32 sequence wraps through one. Every part is a complete JSON object at most 180 bytes before SDK prefixes, respecting the existing 256-byte CDC mirror. Collect all 13 parts before counting a complete window:

| Part | Fields |
|---|---|
| 0 | part_count=13, session_id, codec (3 Opus / 4 G.722), stage |
| 1 | start_us, end_us (device monotonic sample-window time, not host clock) |
| 2 | samples, failures, invalid_duration |
| 3 | sum_us, budget_us=20000, successful |
| 4 | mean_us, min_us, max_us |
| 5 | p95_upper_us, known, over_budget |
| 6 | expected_core, observed_core, mask (valid observed-core bits) |
| 7 | core_wrong, invalid_core, overflow |
| 8 | hist0 (first five bucket counts) |
| 9 | hist1 (last four bucket counts) |
| 10 | heap_internal, heap_min_internal |
| 11 | heap_psram, stack_min |
| 12 | resource_at_us, telemetry_drop |

Missing parts are partial diagnostics, not successful measurement windows. Unknown identity remains unknown. Synthetic fixtures, host-helper tests and firmware builds do not prove on-device timing, CPU/RAM efficiency, ASR precision or acoustic quality.

### Immutable command snapshot for strict hosts

The legacy command `PERF <token> <stage> <part>` remains byte-compatible: it
reads the newest cached window for every request. It is useful for a live
monitor, but a slow 13-part read can span two logger updates. A strict host
must instead use `PERF_SNAPSHOT <token> <stage> <part>` with one nonzero token
and one stage for parts 0 through 12. Part 0 captures one already-published
record in a static 216-byte lease; the same part-0 request is idempotent and
parts 1–12 format that same record. A mismatched token or stage returns
`ERR PERF_SNAPSHOT token=<token> snapshot_lease_mismatch`; discard all parts
and retry from part 0 with a fresh token. This adds no producer work, heap
allocation, codec work, audio-path wait, or wire change to the legacy command.

The existing DSP version 2 codec control, capability intersection, independent L/R codecs, recommended SDK output buffers, fixed 400-byte normal/350-byte repair packets, CRC/session isolation, recovery deadlines and one-audio-event-per-poll scheduling remain intact. Public firmware version is 0.5.14 and image version is app-0.5.14; no automatic flash or serial operation occurs during builds.


## 2026-10-12 bounded capture and asynchronous analysis

This historical milestone predates the dedicated frame-dispatch worker;
current ownership and timing scope are defined above. At that milestone Core 1
had a priority-6 DMA consumer (4096-byte stack), priority-5 DSP/encoder owner
(existing 40960-byte stack), and priority-2 diagnostic worker (8192-byte stack).
Communication, Dongle decode/USB, OTA and packet formats are unchanged.

The capture queue copies four complete 48 kHz stereo 20 ms blocks (3840 PCM bytes
each; at most 80 ms queued). Copying makes ownership explicit; no borrowed
mutable PCM is used asynchronously. A depth-one diagnostic queue never blocks
audio; diagnostics may drop. All queues/buffers/tasks are allocated before the
startup-ready signal. Work buffers request internal RAM; diagnostic scratch
prefers PSRAM with an internal fallback. Queue storage/extra stacks add bounded
internal-memory demand; hardware free/largest-block and stack margins remain
unmeasured. This is not a zero-copy or DMA replacement implementation.

Only the IO task reads active DMA. Suspend/resume and warmup serialize with it
using an IO mutex, and an independent IO-running gate prevents reads before
resume completes. Partial reads, including timeout with returned bytes, retain
complete stereo samples. Invalid IO or a full capture queue stops capture with
an explicit log instead of silently creating a compressed recording timeline.
Buffered blocks are discarded on this failure/suspend boundary. Re-enable uses
the existing hardware warmup and DSP reset. This fail-closed overload policy
requires user re-enable; it is not lossless capture under sustained overload.

At that milestone DSP configuration and both codec process/reset operations shared one owner.
The diagnostic worker exclusively owns FFT state. It resets on raw sequence
gaps/generation changes and rejects results from a previous generation.
Diagnostic results are NOT attached to a later audio packet: audio-packet
analysis is version zero (unavailable). Background diagnostic logs/stats are
sampled results; HMI must not interpret unavailable metadata as healthy spatial
evidence. No new telemetry wire extension is included in this change.

Future audio stages must preserve stereo layout/sample count and frame identity,
allocate state before activation, avoid IO/heap allocation in process calls and
fit the whole 20 ms pipeline deadline. At that milestone the work-over-20ms
counter included the synchronous encoder callback; the current local DSP
counter excludes asynchronous encoding, DMA waits and FFT but includes preemption. Queue
high-water, overload stops and diagnostic drops are cumulative since boot.
Buffers absorb bursts only; no AGC/AEC/AI/beamforming has been added.

Historical validation: the then-current ESP-IDF toolchain ESP32-S3 build and 17 host tests passed in the HMI task
stage. Actual capture/diagnostic C tasks are exercised with deterministic IO/
queue stubs; existing FIR/VAD PCM comparison and codec rollback/affinity tests
are retained. Runtime-codec test uses uint8_t for the bounded wire-item loop,
fixing an existing MSVC /WX narrowing warning without relaxing assertions.
No device run, audio playback, OTA/flash, measured CPU reduction or ASR gain.
Evidence: D:/project/HMI/scratch/firmware-pipeline-20261012.


## 2026-10-12 capture state and IO failure follow-up
Resume only publishes ACTIVE after enable and eight complete warmup reads succeed, and only if capture is still requested. Warmup failure and failed enable publish a fault; already-disabled suspend is idempotent. Control waits for the desired state OR fault, clears old transition bits, and uses a 1500ms timeout. Timeout is unknown completion, not permission to blindly resend.
DMA RX queue overflow is counted in an IRAM callback and stops capture in task context. Three consecutive empty reads also stop capture; partial-timeout data remains preserved. DSP item14 now reflects both requested capture and actual IO-running gate. Background diagnostics include cumulative DMA overflow count. No fault-reason wire extension or HMI UI change was added.
Historical validation: the then-current ESP-IDF toolchain Scepter build, complete host regression and new resume/ACK fault injections. Device scheduling, stack margins and audio continuity remain untested; no flash performed. Evidence: D:/project/HMI/scratch/firmware-state-fix-20261012.

## 2026-10-13 telemetry completion (historical 0.5.10 milestone)

This is project `dual_end_runtime` telemetry using ESP-IDF/FreeRTOS counters,
not an ESP OpenTelemetry SDK. Host export may translate complete records into
OpenTelemetry separately. Existing OTA, pairing, HID and codec packets do not
change. CMake public version and both role headers are numeric `0.5.14`; the
existing `app-`/`factory-` image-key prefix remains an OTA identity convention.
`SOURCE_MANIFEST.json` is historical import metadata, not the new runtime image
identity; actual app description and image hash identify a new build.

Core0's low-priority runtime logger automatically emits the existing ten-part
`audio.device_resources` schema 1 every five seconds, including idle/unpaired
periods; boot identity repeats every thirty seconds without codec activity.
Resource sampling, JSON formatting and serial output remain outside audio
processing. The four-slot perf queue still uses zero-wait publication.
Unsupported CPU accounting, SMP builds and the first idle-counter observation
report `null`; zero CPU must never be inferred from unavailable counters.

`audio.spatial` schema 2 has fifteen parts; every part repeats `boot_id`, `role`,
`session_id`, `seq`, and `part`. Part 0 has `part_count=15` and `sampled_us`; part
1 has nullable bound `codec` and `generation`. Remaining parts contain the
requested/applied mode, layout, fallback, alignment/correlation, current and
cumulative mean mix, learning/frozen/mixed frame counters and processing work.
`mix_q16_sum` in part 14 permits an interval mean computed from its delta divided
by `(delta frames * 65535)`. In the new spatial schema 2 and RX schema 1,
uint64 fields are decimal JSON strings to avoid JavaScript's 53-bit precision
limit. Legacy resource schema 1 keeps `sampled_us`/`interval_us` as JSON numbers.
The mic owner copies the snapshot after
recording mic work; only the lower-priority diagnostic task formats its frozen
copy at approximately one Hz. Disabled/failed runtime produces no valid group.

Frame/work counters are cumulative in one spatial generation. Mode/capture
reset changes generation and clears work; codec/session boundaries start a new
analysis scope even if generation did not change. Mode 0 deliberately skips
spatial processing, so zero spatial frames is not zero microphone capture.
Old five-part schema 1 has no boot/session identity and must remain raw-only.
An incomplete group is unknown, never a zero or valid participation result.

Dongle's existing Core0 RX logger also emits six-part `audio.rx_telemetry`
schema 1. It preserves the RX owner's session, bound decoder codec (null before
first valid binding), activity, snapshot sequence and sampled time. Boot-wide
snapshot/event drop counters saturate at UINT32_MAX. Snapshot age is owner
sampling-to-logger delay in microseconds, not radio latency; stack values are
lifetime byte watermarks. Missing RX/USB telemetry is unknown, not loss-free.

All new JSON payloads fit 180 bytes plus NUL in 181-byte buffers, preserving the
256-byte CDC response budget. Drop types remain separate: diagnostic queue,
perf snapshot queue, RX snapshot/event queues, audio/DMA/USB counters, and host
collector multipart loss. Merge only complete groups, deduplicate by identity,
and calculate deltas inside a single boot/session/codec/generation. Device clocks
remain unsynchronized to host: host receipt time is not precise acoustic time.

Host checks: `python tests/test_telemetry_completion.py -v` uses a native host C
compiler to test real formatting, CPU availability, idle logger cadence,
session pressure, owner snapshot copying and nonblocking RX drop saturation.
These checks and firmware builds do not prove device overhead, continuity or
ASR improvement; real hardware remains separate acceptance.
