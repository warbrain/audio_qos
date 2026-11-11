# Runtime dual-microphone codecs

## DAS + Wiener post-filter (introduced in 0.5.16; telemetry fix in 0.5.17)

Spatial mode 4 fuses the aligned microphone channels through the existing DAS
path, then applies a fixed-memory single-channel spectral Wiener post-filter
before the existing codec dispatch. The shared spatial capability mask uses
bit 3 (`8`) for Wiener and still requires bit 0 (`1`) for DAS; support for mode
4 therefore requires `(mask & 9) == 9`. Mode 4 reports fused mono layout 1.
Modes 0 through 3 retain their prior processing, capability and layout rules.

The filter uses a 512-sample Hann STFT at 16 kHz with a 256-sample hop. Version
0.5.17 counts each completed STFT hop in `processed_hops`, including startup
padding; this is processing-work telemetry and does not count emitted PCM hops.
The signal processing is unchanged. The filter
tracks per-bin noise power from the signal itself with asymmetric smoothing,
clamps spectral gain to 0.15..1.0, and duplicates the filtered mono output to
both PCM channels. It allocates no heap, creates no task, and uses no VAD. The
512-sample startup queue adds 32 ms of latency; state resets with a mode change.
Host synthetic tests establish queue continuity, finite/clamped output and a
small stationary-noise SNR improvement for the test signal only. They do not
establish ESP32 worst-case processing time, real-room denoising, or ASR gains.

## Independent stereo Wiener (mode 5, added in 0.5.17)

Mode 5 sends each physical input channel through a separate instance of the
same 512-point, 256-hop Wiener filter and preserves stereo layout 0. Each
channel has independent noise-power, gain, overlap and queue state. It does not
run DAS, align or average the channels, or replace one channel with the other.
The mode requires capability bit 4 (`16`), separate from mode 4's DAS+Wiener
bits 0 and 3 (`9`). Both filter states reset at the existing PCM mode boundary;
each reports its own noise-valid, processed-hop, underflow and overflow counts.
The two channels are processed sequentially by the realtime owner, so the
additional CPU and memory cost must be measured on-device. Host tests verify
channel isolation and state continuity, not ESP32 timing or ASR improvement.

## Independent NLMS stereo (0.5.14)

DSP payload version 3 retains items 17..22. Item 17 now returns the intersection
of both ends' spatial capabilities: bit 0 = DAS, bit 1 = NLMS + DAS,
bit 2 = independent NLMS stereo. The new mode requires bit 2; an older mask 3
does not enable it. Item 18 selects mode 3, item 19 reports applied mode 3,
and item 20 reports layout 0 (separate stereo). Numeric modes 0/1/2 retain their
existing behavior and layout. The Dongle bounds-checks mode writes and masks
the Scepter capability reply with its own implementation capabilities.

Mode 3 uses the same 64-tap right-to-left NLMS matcher and causal fractional
delay/polarity correction as mode 2. Output left is the aligned left reference;
output right is the matched right, blended back to aligned right while matching
is frozen. It never averages the microphones. Faulty channels remain visible
on their own output instead of being replaced by the other microphone; legacy
fused modes retain their healthy-channel fallback. Low energy, weak correlation,
clipping and coefficient guards retain the existing freeze/fade/reset behavior.

The capture owner applies the mode at a complete PCM boundary, resets matching
and encoder histories, discards queued old-layout audio, and establishes a new
recovery session. Layout 0 always makes two independent G.722/Opus encoder
calls, even if the matched channels temporarily contain equal samples. No host
NLMS or host beamforming is reintroduced. The UAC format remains synchronized
16 kHz PCM16 stereo; downstream host mono downmix remains an independent step.

`audio.spatial` schema 2 and `audio.pipeline` retain their existing fields;
requested/applied mode 3 with layout 0 identifies this mode. Matching counters
still measure right-channel coefficient updates/blending, not DAS activity.
Host synthetic tests cover convergence, no averaging, separate fault channels,
legacy PCM/state equality, control/apply/timeout/capability gates and two codec
calls. They do not establish live acoustic quality, ASR precision or CPU cost.

This component adds two independent mono Opus states to the existing G.722 dual-microphone transport. It does not change the host USB format: synchronized PCM16 stereo, 16 kHz, 20 ms / 320 samples per channel.

## Codec settings

- Default: G.722 codec value 4, packed 64 kbps per microphone, 160 bytes per microphone / 320 bytes per frame.
- Opus: codec value 3, two mono encoders, 16 kHz / PCM16, 20 ms, 32 kbps CBR per microphone, restricted LOWDELAY application (CELT-only), complexity 0, FEC/DTX/VBR disabled. Exact output is checked against 80 bytes per microphone.
- Each encoder uses its SDK-recommended output frame size (bounded to 80..1275 bytes) for a persistent scratch buffer. The radio packet receives only the checked 80-byte actual CBR output. Scratch is freed with its encoder on failed setup/reset.
- Configure rate, expected frame sizes and complexity in both copies of `dual_end_link_config.h`. `dual_end_opus.c` explicitly selects application and optional features. Both header copies must remain identical.
- These are codec settings, separate from device VAD, independent gains, high-pass/FIR, soft limiting and host ASR processing.

## Audio payload and recovery

Normal packets remain 400 bytes. The 320-byte audio reservation, hardware/input offsets, capture index, analysis metadata, CRC coverage, 350-byte repair packet and existing deadlines are unchanged. This first implementation does not reduce wireless packet bandwidth.

For G.722, the 320-byte audio area is unchanged. For Opus, bytes 0..1 contain left packet length 80 (uint16 LE); bytes 2..3 contain right length 80; bytes 4..83 contain left Opus; bytes 84..163 contain right Opus; bytes 164..319 are zero. `current_encoded_bytes` is 164 (four length bytes plus both actual packets), and `previous_encoded_bytes` is zero. The receiver validates both lengths and padding before either decoder runs.

A codec change is executed by the microphone owner at a complete 20 ms PCM boundary under the sender mutex. Target encoders are prepared before commit. Queued originals, recovery cache and both encoder histories are reset, and a fresh random session is established using the existing BEGIN/ACK gate. Failure before preparation leaves the old codec active; failure after reset rolls back to the previous codec with another new session.

The Dongle binds a codec only after the first frame passes the real recovery queue admission (CRC, lengths, session, time and window). All later frames and repairs in that session must use that codec. A new BEGIN resets both decoders and USB session state. Retired sessions, stale frames and repairs cannot bind or alter a new decoder.

Loss concealment preserves the existing 5 ms fade followed by zeros. Both Opus predictor states reset on a missing frame or either-channel decode failure. This implementation does not use Opus PLC or in-band FEC.

## DSP command extension

Existing USB bcdDevice 0101, Report ID 3, 16-byte Feature payload, outer 0x11 and radio control command 0x0B are retained. Values use uint16 LE.

- Items 0..14 must use DSP payload version 1, preserving prior DSP behavior.
- Item 15 (read/write codec) and item 16 (read-only capabilities) must use DSP payload version 2. Both firmware ends validate the item/version combination.
- Item 15 returns 3 for Opus and 4 for G.722. It is session-only and resets to G.722 after boot; no NVS change.
- Item 16 returns the intersection of Scepter encoder and Dongle decoder software capabilities: bit 0 / value 1 = G.722, bit 1 / value 2 = Opus, value 3 = both.
- Old firmware rejects version 2, so an old Dongle forwarder cannot appear to support Opus. Read capabilities successfully before enabling codec switching.
- SET replies contain the actual applied codec after preparation. HMI should GET again to verify. A timeout is an unknown result; read again before another SET.

Radio DSP payload: `[version, op, item, token32 LE, value16 LE, status, reserved x3]`, 13 bytes. USB Feature payload: `[0x11, radio payload, reserved x2]`, 16 bytes plus HID Report ID 3. op 1 = GET, op 2 = SET. status 0 = applied, 1 = pending (Dongle local), 2 = invalid/failed preparation, 3 = apply timeout, 4 = offline/queue error (Dongle local). Matching version/op/item/token and zero reserved bytes are required.

## Runtime budget and verification limits

The microphone encoder owner stack is increased from 6144 to 40960 bytes; the active Dongle RX/decoder owner stack from 8192 to 20480 bytes. These follow the esp_audio_codec 2.6.1 README guidance for all encoders/decoders. With public firmware 0.5.4, the Scepter 40960-byte stack is allocated in PSRAM when the SDK configuration supports external task stacks; its TCB remains internal. Non-PSRAM configurations retain the internal allocation fallback. Dongle retains its existing 20480-byte internal RX stack. Stack sizes, priorities, affinity and the audio wire format are unchanged. The intentional Opus application change from VOIP to restricted LOWDELAY removes the SILK speech-analysis path; it does not increase the 20 ms processing budget or recovery deadlines. Codec heaps are allocated only when their encoders/decoders are prepared; prepared inactive states are kept to avoid reallocating every switch. Two output scratch buffers also use the SDK-recommended capacities (at most 2550 bytes total). Actual free heap and task stack minima still require hardware measurement.

Vendor example numbers are not measurements of this configuration: its Opus table is 48 kHz stereo, 90 kbps, complexity 0, showing 29.4 KB encoder heap / 26.6 KB decoder heap; two separate mono instances at 16 kHz have different overhead. Do not present those values or the table CPU percentages as tested device costs.

The sender logs `codec` and `encode_max_us`. Existing Dongle telemetry logs `decode_frame_max_us` and `rx_stack_min_bytes`. The public firmware version is `0.5.4`. The application image version is `app-0.5.4`: the image-key prefix is retained for existing OTA validators. Build identity is the actual ELF SHA independently reported by Scepter startup/BOOTSTATUS. USB descriptor 0101 remains unchanged.

Microphone startup waits for all three capture buffers to be ready before the mandatory hardware startup returns. Allocation failure frees buffers and is cleaned up by the starter using the matching task deletion API. A readiness timeout is propagated to the existing fatal startup check; no task in the middle of malloc is forcibly deleted. Optional timing telemetry starts after the original mandatory initialization and image confirmation, and publishes its queue only after the logger exists.

Host tests in `tests/test_runtime_codec.py` run the actual wire validator, extracted transaction gates with ESP failure stubs, and the real recovery state machine. Two-end ESP-IDF builds validate API/header/link compatibility. These checks are not audio quality, ASR accuracy, on-device RAM/CPU or acoustic tests. Do not flash or send device commands as part of a build.

## Restricted-low-delay profile repair

Passive measurements of the earlier VOIP profile on the Scepter showed about 29 frames/s instead of the required 50 frames/s, with approximately 20 ms for both encoders and 33 ms for the full microphone processing pipeline. Continuous host callbacks contained repeated synchronized all-zero audio intervals. These observations identify producer throughput below the fixed transport rate; they are not measurements of this revised profile.

The SDK's public `ESP_OPUS_ENC_APPLICATION_LOWDELAY` setting maps to standard Opus restricted-low-delay application 2051. This disables the SILK/hybrid modes and uses CELT. It preserves two independent mono channels, 16 kHz, 20 ms, 32 kbps/channel CBR, complexity 0 and disabled FEC/DTX/VBR. Both mono packets must still contain exactly 80 bytes; recommended-capacity scratch, strict lengths/padding, CRC, session isolation, recovery timing, hardware offsets and G.722 boot default remain in effect. The decoder reads each standard packet's TOC; no proprietary receiver mode or HID protocol change is introduced.

This is a computational-profile change, not proof of improved audio or ASR quality. Hardware acceptance must verify approximately 50 frames/s, the full pipeline's wall-time budget (including preemption), no recurring all-zero intervals, and paired speech/ASR quality against G.722 and the earlier profile. Desktop codec timing or stub tests cannot establish ESP32 performance. Cache sizes and the 40 KiB microphone stack are retained; codec states are not forced into scarce internal RAM.

`tests/test_opus_profile_realtime.py` compiles the actual wrapper with explicit SDK mocks and exercises configuration, both-channel preparation failures, persistent scratch cleanup, strict 80-byte output and decoder packet compatibility. No codec DSP, device connection, recording or playback occurs in these tests.
