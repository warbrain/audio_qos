#include "mic_audio.h"
#include "dual_end_runtime.h"
#include "dual_end_telemetry_format.h"
#include "mic_observability_telemetry.h"
#include "mic_pipeline_telemetry.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_pdm.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "dual_end_link_config.h"
#include "dual_end_audio.h"
#include "mic_analysis.h"
#include "dual_end_spatial.h"

static const char *TAG = "mic_audio";

#define MIC_CAPTURE_SAMPLE_RATE_HZ 48000
#define MIC_OUTPUT_SAMPLE_RATE_HZ 16000
#define MIC_CHANNEL_COUNT 2
#define MIC_FRAME_MS 20
#define MIC_DECIMATION_FACTOR 3
#define MIC_CAPTURE_FRAMES_PER_PACKET \
    ((MIC_CAPTURE_SAMPLE_RATE_HZ * MIC_FRAME_MS) / 1000)
#define MIC_OUTPUT_FRAMES_PER_PACKET \
    ((MIC_OUTPUT_SAMPLE_RATE_HZ * MIC_FRAME_MS) / 1000)
#define MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET \
    (MIC_OUTPUT_FRAMES_PER_PACKET * MIC_CHANNEL_COUNT)
#define MIC_CAPTURE_BUFFER_BYTES \
    (MIC_CAPTURE_FRAMES_PER_PACKET * MIC_CHANNEL_COUNT * sizeof(int16_t))

#define MIC_I2S_NUM I2S_NUM_0
#define MIC_I2S_DMA_DESC_NUM 4
#define MIC_I2S_DMA_FRAME_NUM MIC_CAPTURE_FRAMES_PER_PACKET
#define MIC_PDM_CLK_GPIO 14
#define MIC_PDM_DATA_GPIO 47
#define MIC_PDM_CLK_HZ (MIC_CAPTURE_SAMPLE_RATE_HZ * 64)

#define MIC_HPF_CUTOFF_HZ 80
#define MIC_HPF_ALPHA 0.9895826647f
#define MIC_DECIMATOR_FIR_TAPS 95
#define MIC_DECIMATOR_FIR_STORAGE 96
#define MIC_DECIMATOR_CUTOFF_HZ 7000

/* Apply one fixed post-filter gain to both microphones. The build-time default
 * comes from Kconfig and a validated runtime override can be stored in NVS.
 * Taking one gain snapshot per stereo sample keeps the L/R relationship intact.
 * The old stateful
 * limiter used an instantaneous attack and a 100 ms release. A short impulse
 * could therefore duck speech long after the impulse had ended. The new
 * memoryless soft knee only compresses samples while the overload exists and
 * still applies one shared gain to preserve stereo amplitude and phase. */
#define MIC_SOFT_LIMIT_KNEE 20000
#define MIC_SOFT_LIMIT_CEILING 26000
#define MIC_AUDIO_PROFILE "Scepter_double_runtime_codec_session_dsp"
#define MIC_CONFIG_NVS_NAMESPACE "mic_cfg"
#define MIC_CONFIG_NVS_GAIN_KEY "fixed_gain_x"

#define MIC_READ_TIMEOUT_MS 50
#define MIC_STATE_TIMEOUT_MS 1500
#define MIC_STATE_FAULT_BIT BIT4
#define MIC_RESUME_SETTLE_MS 10
#define MIC_STATE_ACTIVE_BIT BIT0
#define MIC_STATE_SUSPENDED_BIT BIT1
#define MIC_START_READY_BIT BIT2
#define MIC_START_FAILED_BIT BIT3
#define MIC_START_TIMEOUT_MS 3000
#if CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
#define MIC_TASK_PSRAM_STACK 1
#else
#define MIC_TASK_PSRAM_STACK 0
#endif
/* esp_audio_codec recommends about 40 KiB for its encoder owner task. */
#define MIC_TASK_STACK_BYTES (40 * 1024)
#define MIC_ANALYSIS_RUNTIME_LOG_INTERVAL_MS 5000

_Static_assert(MIC_CAPTURE_SAMPLE_RATE_HZ / MIC_OUTPUT_SAMPLE_RATE_HZ ==
                   MIC_DECIMATION_FACTOR,
               "microphone decimation ratio must remain 3:1");
_Static_assert(MIC_CAPTURE_FRAMES_PER_PACKET / MIC_DECIMATION_FACTOR ==
                   MIC_OUTPUT_FRAMES_PER_PACKET,
               "20 ms microphone frame sizes must match the decimator");
_Static_assert(DUAL_END_SPATIAL_TAPS == 64U,
               "Scepter NLMS observability must not change the validated 64-tap matcher");
_Static_assert(MIC_OUTPUT_SAMPLE_RATE_HZ == DUAL_END_AUDIO_SAMPLE_RATE_HZ,
               "microphone output rate must match the wire protocol");
_Static_assert(MIC_CHANNEL_COUNT == DUAL_END_AUDIO_CHANNELS,
               "microphone channel count must match the wire protocol");
_Static_assert(MIC_OUTPUT_FRAMES_PER_PACKET == DUAL_END_AUDIO_FRAME_SAMPLES,
               "microphone frame size must match the wire protocol");
_Static_assert(CONFIG_MIC_OUTPUT_GAIN_X >= DUAL_END_MIC_GAIN_MIN_X &&
                   CONFIG_MIC_OUTPUT_GAIN_X <= DUAL_END_MIC_GAIN_MAX_X,
               "configured microphone output gain is outside the supported range");
_Static_assert(MIC_SOFT_LIMIT_KNEE > 0 && MIC_SOFT_LIMIT_KNEE < MIC_SOFT_LIMIT_CEILING,
               "microphone soft-limit knee must be below its ceiling");
_Static_assert(MIC_SOFT_LIMIT_CEILING <= INT16_MAX,
               "microphone soft-limit ceiling must fit PCM16");

/* 95-tap Blackman-windowed low-pass, generated for Fs=48 kHz and Fc=7 kHz.
 * The response is about -38 dB at the 8 kHz output Nyquist frequency and more
 * than -75 dB from 8.5 kHz through the first strong alias bands. It runs after
 * the per-channel HPF and before every third sample is emitted. */
static const float s_decimator_coeff[MIC_DECIMATOR_FIR_TAPS] = {
    0.0000000000f,
    -0.0000026920f,
    -0.0000043819f,
    0.0000132790f,
    0.0000484205f,
    0.0000559982f,
    -0.0000154946f,
    -0.0001460663f,
    -0.0002129858f,
    -0.0000792011f,
    0.0002416716f,
    0.0005055925f,
    0.0003860551f,
    -0.0002032456f,
    -0.0008885433f,
    -0.0010103374f,
    -0.0001831607f,
    0.0011845251f,
    0.0019692890f,
    0.0011703875f,
    -0.0010497781f,
    -0.0030895353f,
    -0.0029450610f,
    0.0000000000f,
    0.0039184592f,
    0.0054735245f,
    0.0024802525f,
    -0.0036964137f,
    -0.0083411002f,
    -0.0067569692f,
    0.0014146108f,
    0.0106344384f,
    0.0128485786f,
    0.0040773497f,
    -0.0108737768f,
    -0.0202900591f,
    -0.0140720593f,
    0.0068456627f,
    0.0281325696f,
    0.0306208270f,
    0.0054232128f,
    -0.0351098317f,
    -0.0602840814f,
    -0.0386372236f,
    0.0399390985f,
    0.1526078966f,
    0.2520680726f,
    0.2916644525f,
    0.2520680726f,
    0.1526078966f,
    0.0399390985f,
    -0.0386372236f,
    -0.0602840814f,
    -0.0351098317f,
    0.0054232128f,
    0.0306208270f,
    0.0281325696f,
    0.0068456627f,
    -0.0140720593f,
    -0.0202900591f,
    -0.0108737768f,
    0.0040773497f,
    0.0128485786f,
    0.0106344384f,
    0.0014146108f,
    -0.0067569692f,
    -0.0083411002f,
    -0.0036964137f,
    0.0024802525f,
    0.0054735245f,
    0.0039184592f,
    0.0000000000f,
    -0.0029450610f,
    -0.0030895353f,
    -0.0010497781f,
    0.0011703875f,
    0.0019692890f,
    0.0011845251f,
    -0.0001831607f,
    -0.0010103374f,
    -0.0008885433f,
    -0.0002032456f,
    0.0003860551f,
    0.0005055925f,
    0.0002416716f,
    -0.0000792011f,
    -0.0002129858f,
    -0.0001460663f,
    -0.0000154946f,
    0.0000559982f,
    0.0000484205f,
    0.0000132790f,
    -0.0000043819f,
    -0.0000026920f,
    0.0000000000f,
};

static i2s_chan_handle_t s_mic_rx_chan;
static TaskHandle_t s_mic_task_handle;
static EventGroupHandle_t s_mic_state_events;
static volatile bool s_capture_enabled = true;
static volatile bool s_capture_io_running;
static volatile bool s_motor_active;
static volatile esp_err_t s_state_change_result = ESP_OK;
static volatile uint8_t s_output_gain_x = CONFIG_MIC_OUTPUT_GAIN_X;
static volatile uint8_t s_requested_output_gain_x = CONFIG_MIC_OUTPUT_GAIN_X;
static mic_audio_frame_cb_t s_frame_callback;
static void *s_frame_user_ctx;
static mic_audio_stats_cb_t s_stats_callback;
static void *s_stats_user_ctx;
/* A queued callback carries this generation.  Config/capture resets advance it
 * so the worker can discard old PCM without touching a queue in flight. */
static uint32_t s_frame_dispatch_epoch = 1U;
/* A short spin-copy lock protects this cross-core stale-frame generation. It
 * is never the sender mutex and never surrounds codec/radio work. */
static portMUX_TYPE s_frame_dispatch_epoch_lock = portMUX_INITIALIZER_UNLOCKED;

/* Audio task owns DSP state. Control task publishes settings under a short lock.
 * New settings are session-only; legacy 0x0A still persists its shared gain. */
static portMUX_TYPE s_config_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_config[13] = {1,0,CONFIG_MIC_OUTPUT_GAIN_X,CONFIG_MIC_OUTPUT_GAIN_X,1,80,1,7000,1,20000,26000,48,300};
static uint16_t s_requested_config[13] = {1,0,CONFIG_MIC_OUTPUT_GAIN_X,CONFIG_MIC_OUTPUT_GAIN_X,1,80,1,7000,1,20000,26000,48,300};
static uint32_t s_config_generation, s_applied_generation;
_Alignas(16) static float s_runtime_fir[MIC_DECIMATOR_FIR_STORAGE];
static float s_runtime_hpf = MIC_HPF_ALPHA;
static uint32_t s_vad_hold_frames;
static float s_vad_gain = 1.0f;
static bool s_vad_open = true;
static void mic_owner_apply_dsp_config(void);
/* Only asynchronous transition/lifecycle code takes this control lock.  The
 * 20 ms DSP owner never waits on it; the dispatch and config workers hold it
 * while mutating sender state so a codec/layout transition cannot relabel
 * queued PCM. */
static void mic_frame_dispatch_control_lock(void);
static void mic_frame_dispatch_control_unlock(void);
static void mic_frame_dispatch_reset_stale_queue(void);
static uint8_t s_requested_codec = DUAL_END_AUDIO_CODEC_G722;
static uint32_t s_codec_generation, s_codec_applied_generation;
static esp_err_t s_codec_apply_result = ESP_OK;
/* Stop publishing PCM callbacks while the config task waits for the sender
 * mutex.  Otherwise the 20 ms owner can fill its deliberately shallow queue
 * during an Opus/G.722 reconfiguration. */
static bool s_codec_transition_active;
static TaskHandle_t s_config_task_handle;
/* Sole owner of s_spatial is mic_audio_task, never DMA/IO/HID/config worker.
 * The config worker changes only sender transport state and posts a compact
 * commit for this owner to apply at the next 20 ms boundary. */
static dual_end_spatial_t *s_spatial;
static dual_end_spatial_stats_t s_spatial_status;
static uint16_t s_spatial_requested_mode;
static uint32_t s_spatial_request_generation, s_spatial_applied_request;
static uint16_t s_spatial_generation;
static esp_err_t s_spatial_apply_result = ESP_OK;
/* One in-flight spatial transition preserves the order between sender layout
 * reconfiguration and the owner-local matcher reset.  The owner drops an
 * affected 20 ms frame rather than waiting for this lower-priority worker. */
static bool s_spatial_transition_active, s_spatial_dispatch_paused, s_spatial_commit_pending;
static uint32_t s_spatial_transition_generation, s_spatial_commit_generation;
static uint16_t s_spatial_commit_mode;
static uint64_t s_spatial_work_us;
static uint32_t s_spatial_work_frames, s_spatial_work_max_us;
/* One complete audio-owner snapshot per PCM block.  Sender admission now runs
 * on the bounded frame-dispatch worker, so the audio owner only consumes a
 * completed timing event before publishing this identity.  Logger never locks
 * or inspects the encoder. */
static dual_end_spatial_telemetry_t s_spatial_telemetry;
typedef struct {
    uint64_t updated_frames;
    uint64_t frozen_frames;
    uint32_t bounded_updates;
    uint32_t resets;
    uint32_t update_streak;
    bool converged;
} mic_nlms_observer_t;
static mic_nlms_observer_t s_nlms_observer;

static void mic_nlms_observer_reset(void)
{
    memset(&s_nlms_observer, 0, sizeof(s_nlms_observer));
}

/* The spatial matcher deliberately remains untouched. These counters are
 * derived from its completed-frame public statistics in the same owner task.
 * Convergence has 20 ms resolution: first_converged_frame is the completed
 * PCM block boundary where mix first reaches one. */
static mic_nlms_block_t mic_nlms_observe(const dual_end_spatial_stats_t *stats)
{
    mic_nlms_block_t block = {0};
    if (!stats || (stats->mode != DUAL_END_SPATIAL_NLMS &&
                   stats->mode != DUAL_END_SPATIAL_NLMS_STEREO)) return block;
    block.valid = true;
    block.reason = stats->fallback;
    const uint64_t updated = stats->updated_frames >= s_nlms_observer.updated_frames ?
        stats->updated_frames - s_nlms_observer.updated_frames : stats->updated_frames;
    const uint64_t frozen = stats->frozen_frames >= s_nlms_observer.frozen_frames ?
        stats->frozen_frames - s_nlms_observer.frozen_frames : stats->frozen_frames;
    const uint32_t bounded_updates = stats->bounded_updates >= s_nlms_observer.bounded_updates ?
        stats->bounded_updates - s_nlms_observer.bounded_updates : stats->bounded_updates;
    const uint32_t resets = stats->resets >= s_nlms_observer.resets ?
        stats->resets - s_nlms_observer.resets : stats->resets;
    block.updated_samples = updated > UINT32_MAX ? UINT32_MAX : (uint32_t)updated;
    block.frozen_samples = frozen > UINT32_MAX ? UINT32_MAX : (uint32_t)frozen;
    block.bounded_updates = bounded_updates;
    block.resets = resets;
    if (block.updated_samples) {
        s_nlms_observer.update_streak = s_nlms_observer.update_streak == UINT32_MAX ?
            UINT32_MAX : s_nlms_observer.update_streak + 1U;
    } else {
        s_nlms_observer.update_streak = 0;
    }
    block.update_streak = s_nlms_observer.update_streak;
    block.converged = stats->matching_mix >= 1.0f;
    block.convergence_event = block.converged && !s_nlms_observer.converged;
    if (block.convergence_event) {
        /* dual_end_spatial_process increments frames after its 20 ms block;
         * preserve that positive ending index. The fallback only protects a
         * malformed diagnostic snapshot and never changes matcher state. */
        block.convergence_frame = stats->frames ? stats->frames :
            DUAL_END_SPATIAL_FRAME_SAMPLES;
        block.has_convergence_frame = true;
    }
    s_nlms_observer.updated_frames = stats->updated_frames;
    s_nlms_observer.frozen_frames = stats->frozen_frames;
    s_nlms_observer.bounded_updates = stats->bounded_updates;
    s_nlms_observer.resets = stats->resets;
    s_nlms_observer.converged = block.converged;
    return block;
}

static void mic_spatial_cache_snapshot(void)
{
    uint32_t session=0;uint8_t codec=0;uint64_t sampled=0;
    const bool known=dual_end_runtime_owner_context(DUAL_END_STAGE_MIC_PIPELINE,
        &session,&codec,&sampled);
    dual_end_spatial_telemetry_t snapshot={.boot_id=dual_end_runtime_boot_id(),
        .role=0,.session_id=known?session:0,.codec=codec,.codec_known=known,
        .sampled_us=known?sampled:(uint64_t)esp_timer_get_time()};
    portENTER_CRITICAL(&s_config_lock);
    const dual_end_spatial_stats_t *stats=&s_spatial->stats;
    snapshot.generation=stats->generation;snapshot.requested=s_spatial_requested_mode;
    snapshot.applied=stats->mode;snapshot.layout=stats->layout;snapshot.fallback=stats->fallback;
    snapshot.polarity_inverted=stats->polarity_inverted;
    snapshot.delay_samples=stats->delay_samples;snapshot.correlation=stats->correlation;
    snapshot.current_mix=stats->matching_mix;snapshot.frames=stats->frames;
    snapshot.learned_frames=stats->updated_frames;snapshot.frozen_frames=stats->frozen_frames;
    snapshot.mixed_frames=stats->mixed_frames;snapshot.nlms_fallback_frames=stats->fallback_frames;
    snapshot.mix_q16_sum=stats->matching_mix_q16_sum;snapshot.bounded_updates=stats->bounded_updates;
    snapshot.resets=stats->resets;snapshot.processing_us=s_spatial_work_us;
    snapshot.processing_frames=s_spatial_work_frames;snapshot.processing_max_us=s_spatial_work_max_us;
    s_spatial_telemetry=snapshot;
    portEXIT_CRITICAL(&s_config_lock);
}

static void mic_frame_dispatch_invalidate(void)
{
    /* Zero is never a special valid epoch.  Wrap to one so a very old queued
     * frame cannot become current after the 32-bit counter rolls over. */
    portENTER_CRITICAL(&s_frame_dispatch_epoch_lock);
    s_frame_dispatch_epoch = s_frame_dispatch_epoch == UINT32_MAX ? 1U :
        s_frame_dispatch_epoch + 1U;
    portEXIT_CRITICAL(&s_frame_dispatch_epoch_lock);
}

static uint32_t mic_frame_dispatch_epoch_load(void)
{
    portENTER_CRITICAL(&s_frame_dispatch_epoch_lock);
    const uint32_t epoch=s_frame_dispatch_epoch;
    portEXIT_CRITICAL(&s_frame_dispatch_epoch_lock);
    return epoch;
}

/* This touches matcher state, so only the 20 ms DSP owner may call it.  It
 * deliberately does not take the sender/control mutex. */
static void mic_spatial_reset_state_owner(uint16_t mode) {
    dual_end_spatial_reset(s_spatial,mode,++s_spatial_generation);
    mic_nlms_observer_reset();
    mic_frame_dispatch_invalidate();
    portENTER_CRITICAL(&s_config_lock);
    s_spatial_status=s_spatial->stats;
    s_spatial_work_us=0;s_spatial_work_frames=0;s_spatial_work_max_us=0;
    memset(&s_spatial_telemetry,0,sizeof(s_spatial_telemetry));
    portEXIT_CRITICAL(&s_config_lock);
}

static void mic_config_transition_notify(void)
{
    TaskHandle_t task = s_config_task_handle;
    if (task) xTaskNotifyGive(task);
}

/* The config worker owns sender changes.  It commits a successful layout
 * transition back to the DSP owner rather than ever resetting s_spatial from a
 * different task.  There is exactly one in-flight transition, which preserves
 * layout/reset ordering even when a newer request arrives while sender setup
 * is waiting on its own mutex. */
static bool mic_spatial_apply_control(void)
{
    portENTER_CRITICAL(&s_config_lock);
    uint32_t generation=s_spatial_request_generation;
    uint16_t mode=s_spatial_requested_mode;
    bool pending=generation!=s_spatial_applied_request && !s_spatial_transition_active;
    if (pending) {
        s_spatial_transition_active=true;
        s_spatial_transition_generation=generation;
    }
    portEXIT_CRITICAL(&s_config_lock);
    if(!pending)return false;
    /* Drain queued old-layout audio and encoder histories before commit. The
     * existing recovery session ID protects the Dongle from mixing old frames. */
    /* Keep a frame callback and its sender-layout transition mutually
     * exclusive.  The sender may reset its session or encoder before it can
     * report an error, so invalidate every queued frame after an attempted
     * transition, not only after a successful layout change. */
    mic_frame_dispatch_control_lock();
    esp_err_t result=dual_end_audio_sender_set_processing_layout(
        (uint8_t)dual_end_spatial_mode_layout(mode));
    mic_frame_dispatch_invalidate();
    mic_frame_dispatch_reset_stale_queue();
    mic_frame_dispatch_control_unlock();
    portENTER_CRITICAL(&s_config_lock);
    if (s_spatial_transition_active &&
        s_spatial_transition_generation==generation) {
        if (result==ESP_OK) {
            s_spatial_commit_mode=mode;
            s_spatial_commit_generation=generation;
            s_spatial_commit_pending=true;
            /* Keep the barrier armed until the owner resets its matcher at a
             * frame boundary.  It drops the affected packet instead of
             * emitting old-mode PCM through the new layout. */
        } else {
            s_spatial_apply_result=result;
            s_spatial_applied_request=generation;
            s_spatial_transition_active=false;
            s_spatial_dispatch_paused=s_spatial_request_generation!=generation;
        }
    }
    portEXIT_CRITICAL(&s_config_lock);
    return true;
}

/* Called only by mic_audio_task at a PCM-frame boundary.  It is intentionally
 * bounded local DSP work: no sender mutex, codec preparation, radio call, heap
 * action, wait, or log belongs on this path. */
static void mic_owner_apply_spatial_commit(void)
{
    uint32_t generation=0;
    uint16_t mode=0;
    bool pending=false;
    portENTER_CRITICAL(&s_config_lock);
    pending=s_spatial_transition_active && s_spatial_commit_pending &&
        s_spatial_transition_generation==s_spatial_commit_generation;
    if (pending) {
        generation=s_spatial_commit_generation;
        mode=s_spatial_commit_mode;
    }
    portEXIT_CRITICAL(&s_config_lock);
    if (!pending) return;

    mic_spatial_reset_state_owner(mode);

    bool committed=false;
    portENTER_CRITICAL(&s_config_lock);
    if (s_spatial_transition_active && s_spatial_commit_pending &&
        s_spatial_transition_generation==generation &&
        s_spatial_commit_generation==generation) {
        s_spatial_status=s_spatial->stats;
        s_spatial_apply_result=ESP_OK;
        s_spatial_applied_request=generation;
        s_spatial_commit_pending=false;
        s_spatial_transition_active=false;
        s_spatial_dispatch_paused=s_spatial_request_generation!=generation;
        committed=true;
    }
    portEXIT_CRITICAL(&s_config_lock);
    if (committed) mic_config_transition_notify();
}

static bool mic_owner_dispatch_transition_active(void)
{
    portENTER_CRITICAL(&s_config_lock);
    const bool active=s_spatial_transition_active || s_spatial_dispatch_paused ||
        s_codec_transition_active;
    portEXIT_CRITICAL(&s_config_lock);
    return active;
}

static esp_err_t mic_spatial_parameter(bool write,uint8_t item,uint16_t value,uint16_t *actual) {
    if(write && (item!=18 || !dual_end_spatial_mode_supported(value,DUAL_END_SPATIAL_CAPABILITIES) ||
        !s_mic_task_handle || !s_config_task_handle))return ESP_ERR_INVALID_ARG;
    if(write) {
        portENTER_CRITICAL(&s_config_lock);
        s_spatial_requested_mode=value;
        uint32_t generation=++s_spatial_request_generation;
        s_spatial_dispatch_paused=true;
        portEXIT_CRITICAL(&s_config_lock);
        mic_frame_dispatch_invalidate();
        mic_config_transition_notify();
        bool applied=false; esp_err_t result=ESP_ERR_TIMEOUT;
        for(unsigned i=0;i<100;++i) {
            portENTER_CRITICAL(&s_config_lock);
            applied=s_spatial_applied_request==generation;
            if(applied)result=s_spatial_apply_result;
            portEXIT_CRITICAL(&s_config_lock);
            if(applied)break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if(!applied || result!=ESP_OK)return result;
    }
    portENTER_CRITICAL(&s_config_lock);
    switch(item) {
        case 17:*actual=DUAL_END_SPATIAL_CAPABILITIES;break;
        case 18:*actual=s_spatial_requested_mode;break;
        case 19:*actual=s_spatial_status.mode;break;
        case 20:*actual=s_spatial_status.layout;break;
        case 21:*actual=s_spatial_status.fallback;break;
        case 22:*actual=s_spatial_status.generation;break;
        default:portEXIT_CRITICAL(&s_config_lock);return ESP_ERR_INVALID_ARG;
    }
    portEXIT_CRITICAL(&s_config_lock);return ESP_OK;
}

/* The low-priority config worker is the only normal caller.  The pre-start
 * fallback in mic_codec_parameter is retained for the legacy control-plane
 * ordering where no PCM owner exists yet. */
static bool mic_codec_apply_control(void)
{
    portENTER_CRITICAL(&s_config_lock);
    const uint32_t generation = s_codec_generation;
    const uint8_t codec = s_requested_codec;
    const bool pending = generation != s_codec_applied_generation;
    portEXIT_CRITICAL(&s_config_lock);
    if (!pending) return false;
    /* Invalidate the in-flight owner frame as soon as the transition starts;
     * the owner checks this epoch again immediately before queueing. */
    mic_frame_dispatch_invalidate();
    /* Sender reconfiguration and callback dispatch share this transition
     * gate.  A failed codec setup can still have reset the sender session or
     * encoder, so no frame queued before the attempt may reach it afterwards. */
    mic_frame_dispatch_control_lock();
    const esp_err_t ret = dual_end_audio_sender_set_codec(codec);
    mic_frame_dispatch_invalidate();
    /* Frames queued before the epoch change must not occupy slots when the
     * new codec starts.  The sender lock prevents a callback from consuming
     * the queue concurrently; any callback already dequeued will observe the
     * invalidated epoch after this transition. */
    mic_frame_dispatch_reset_stale_queue();
    mic_frame_dispatch_control_unlock();
    portENTER_CRITICAL(&s_config_lock);
    s_codec_apply_result = ret;
    s_codec_applied_generation = generation;
    if (s_codec_generation == generation) s_codec_transition_active = false;
    portEXIT_CRITICAL(&s_config_lock);
    return true;
}

static void mic_config_transition_task_fn(void *arg)
{
    (void)arg;
    while (true) {
        (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        /* Coalesce bursts.  A successful spatial sender transition waits for
         * the owner acknowledgment before applying another layout, preserving
         * the configuration/lifetime ordering without delaying PCM. */
        while (mic_codec_apply_control()) {
        }
        if (mic_spatial_apply_control()) continue;
    }
}

static esp_err_t mic_codec_parameter(bool write, uint16_t value, uint16_t *actual)
{
    if (write && !dual_end_audio_codec_is_opus(value) && value != DUAL_END_AUDIO_CODEC_G722)
        return ESP_ERR_INVALID_ARG;
    if (write) {
        portENTER_CRITICAL(&s_config_lock);
        s_requested_codec = (uint8_t)value;
        const uint32_t generation = ++s_codec_generation;
        s_codec_transition_active = true;
        portEXIT_CRITICAL(&s_config_lock);
        mic_frame_dispatch_invalidate();
        if (s_config_task_handle) mic_config_transition_notify();
        else (void)mic_codec_apply_control();
        bool applied = false;
        esp_err_t ret = ESP_ERR_TIMEOUT;
        for (unsigned i = 0; i < 100; ++i) {
            portENTER_CRITICAL(&s_config_lock);
            applied = s_codec_applied_generation == generation;
            if (applied) ret = s_codec_apply_result;
            portEXIT_CRITICAL(&s_config_lock);
            if (applied) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        *actual = dual_end_audio_sender_get_codec();
        if (!applied || ret != ESP_OK) return ret;
    }
    *actual = dual_end_audio_sender_get_codec();
    return ESP_OK;
}

static float s_hpf_previous_input[MIC_CHANNEL_COUNT];
static float s_hpf_previous_output[MIC_CHANNEL_COUNT];
_Alignas(16) static float s_fir_history[MIC_CHANNEL_COUNT][MIC_DECIMATOR_FIR_STORAGE];
_Static_assert(sizeof(s_fir_history[0]) % 16 == 0, "FIR channel stride alignment");
static size_t s_fir_write_index;
static uint8_t s_decimation_phase;
static uint32_t s_soft_limited_frames;
static float s_soft_limit_min_gain;
static int16_t s_warmup_discard_buffer[MIC_CAPTURE_BUFFER_BYTES / sizeof(int16_t)];

static int16_t clamp_i16_from_float(float value)
{
    if (value > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (value < (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static inline int32_t abs_i16_to_i32(int16_t value)
{
    int32_t v = (int32_t)value;
    return (v < 0) ? -v : v;
}

static bool mic_config_valid(const uint16_t *v) {
    return v[1]<=1 && v[2]>=1 && v[2]<=32 && v[3]>=1 && v[3]<=32 &&
        v[4]<=1 && v[5]>=20 && v[5]<=500 && v[6]<=1 && v[7]>=1000 && v[7]<=7000 &&
        v[8]<=1 && v[9]>=1000 && v[9]<v[10] && v[10]<=32767 &&
        v[11]>=20 && v[11]<=80 && v[12]<=2000;
}
/* The 20 ms owner applies only local filter/gain/VAD state.  Codec and spatial
 * transport transitions are intentionally handled by mic_config_transition_task_fn;
 * see mic_owner_apply_spatial_commit for the owner-side spatial handoff. */
static void mic_owner_apply_dsp_config(void) {
    uint16_t next[13]; uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation=s_config_generation;
    memcpy(next,s_requested_config,sizeof(next));
    portEXIT_CRITICAL(&s_config_lock);
    if (generation==s_applied_generation) return;
    if (next[7]==7000) memcpy(s_runtime_fir,s_decimator_coeff,sizeof(s_decimator_coeff));
    else {
        float sum=0, fc=(float)next[7]/48000.0f;
        for (int i=0;i<MIC_DECIMATOR_FIR_TAPS;i++) {
            float n=(float)(i-47), w=0.42f-0.5f*cosf(2.0f*3.14159265359f*i/94.0f)+0.08f*cosf(4.0f*3.14159265359f*i/94.0f);
            s_runtime_fir[i]=(i==47 ? 2.0f*fc : sinf(2.0f*3.14159265359f*fc*n)/(3.14159265359f*n))*w;
            sum+=s_runtime_fir[i];
        }
        for (int i=0;i<MIC_DECIMATOR_FIR_TAPS;i++) s_runtime_fir[i]/=sum;
    }
    s_runtime_hpf=next[5]==80 ? MIC_HPF_ALPHA : 1.0f/(1.0f+2.0f*3.14159265359f*next[5]/48000.0f);
    s_vad_hold_frames=0; s_vad_gain=1.0f;
    /* A gain/filter/VAD generation changes the learned channel response. */
    mic_spatial_reset_state_owner(s_spatial->stats.mode);
    portENTER_CRITICAL(&s_config_lock);
    memcpy(s_config,next,sizeof(next));
    s_output_gain_x=(uint8_t)next[2];
    s_applied_generation=generation;
    portEXIT_CRITICAL(&s_config_lock);
}
esp_err_t mic_audio_parameter(bool write, uint8_t item, uint16_t value, uint16_t *actual) {
    if (!actual || item > 22) return ESP_ERR_INVALID_ARG;
    if (item >= 17) return mic_spatial_parameter(write,item,value,actual);
    if (item == 15) return mic_codec_parameter(write, value, actual);
    if (item == 16) {
        if (write) return ESP_ERR_INVALID_ARG;
        *actual = DUAL_END_AUDIO_CODEC_CAPABILITIES;
        return ESP_OK;
    }
    if (write && (item == 0 || item > 12)) return ESP_ERR_INVALID_ARG;
    uint32_t generation=0;
    if (write) {
        uint16_t next[13];
        portENTER_CRITICAL(&s_config_lock);
        memcpy(next,s_requested_config,sizeof(next));
        next[item]=value;
        if (!mic_config_valid(next)) { portEXIT_CRITICAL(&s_config_lock); return ESP_ERR_INVALID_ARG; }
        memcpy(s_requested_config,next,sizeof(next)); generation=++s_config_generation;
        portEXIT_CRITICAL(&s_config_lock);
        if (s_mic_task_handle) xTaskNotifyGive(s_mic_task_handle);
        else mic_owner_apply_dsp_config();
        bool applied=false;
        for (unsigned i=0;i<100;i++) {
            portENTER_CRITICAL(&s_config_lock); applied=s_applied_generation==generation; portEXIT_CRITICAL(&s_config_lock);
            if (applied) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!applied) return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_config_lock);
    *actual=item==13 ? (uint16_t)s_vad_open : item==14 ? (uint16_t)(s_capture_enabled && s_capture_io_running) : s_config[item];
    portEXIT_CRITICAL(&s_config_lock);
    return ESP_OK;
}
static void mic_vad_process(int16_t *pcm) {
    /* Off is a true pass-through, including the established hold/open state.
     * Avoid software-double energy and threshold work when it cannot gate. */
    if (!s_config[1]) {
        s_vad_hold_frames=(s_config[12]+19)/20;
        portENTER_CRITICAL(&s_config_lock); s_vad_open=true; portEXIT_CRITICAL(&s_config_lock);
        s_vad_gain=1.0f;
        return;
    }
    /* PCM16 squares and 320-sample sums are exact integers below 2^39.
     * Convert only for the original division, preserving its float rounding. */
    uint64_t energy[2]={0,0};
    for (int i=0;i<MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET;i++) {
        const int32_t sample=pcm[i];
        energy[i%2]+=(uint64_t)(sample*sample);
    }
    const uint64_t maximum_energy=energy[0]>energy[1] ? energy[0] : energy[1];
    float rms=sqrtf((float)((double)maximum_energy/MIC_OUTPUT_FRAMES_PER_PACKET));
    float threshold=32768.0f*powf(10.0f,-(float)s_config[11]/20.0f);
    bool open=!s_config[1] || rms>=threshold;
    if (open) s_vad_hold_frames=(s_config[12]+19)/20;
    else if (s_vad_hold_frames) { s_vad_hold_frames--; open=true; }
    portENTER_CRITICAL(&s_config_lock); s_vad_open=open; portEXIT_CRITICAL(&s_config_lock);
    if (!s_config[1]) { s_vad_gain=1.0f; return; }
    for (int i=0;i<MIC_OUTPUT_FRAMES_PER_PACKET;i++) {
        s_vad_gain=fminf(1.0f,fmaxf(0.0f,s_vad_gain+(open ? 0.0125f : -0.0125f)));
        pcm[i*2]=clamp_i16_from_float(pcm[i*2]*s_vad_gain);
        pcm[i*2+1]=clamp_i16_from_float(pcm[i*2+1]*s_vad_gain);
    }
}


/* Bounded IO/compute separation. Queues copy complete synchronized blocks;
 * producer storage is never handed to an asynchronous consumer by reference.
 * 4 raw blocks = 15 KiB plus queue overhead. DMA remains driver-owned. */
#define MIC_PIPELINE_BLOCKS 4
#define MIC_FRAME_DISPATCH_BLOCKS 2
#define MIC_CAPTURE_TASK_STACK 4096
#define MIC_DIAGNOSTIC_TASK_STACK 8192
#define MIC_TELEMETRY_TASK_STACK 6144
#define MIC_CONFIG_TASK_STACK 4096
/* Separate budgets permit measured tuning without shrinking codec stack. */
#define MIC_FRAME_DISPATCH_TASK_STACK (40 * 1024)
#define MIC_FRAME_DISPATCH_TASK_PRIORITY 4
#define MIC_BACKGROUND_TASK_CORE 0
#define MIC_BACKGROUND_TASK_PRIORITY 1
#define MIC_PROCESSING_BUDGET_US 20000
#define MIC_DIAGNOSTIC_SAMPLE_FRAMES 10U /* 50 Hz source -> 5 Hz diagnostics */
#define MIC_DIAGNOSTIC_MAX_GAP_FRAMES 50U /* Reset after one second without a sample. */
_Static_assert(MIC_PIPELINE_BLOCKS > 0 && MIC_PIPELINE_BLOCKS <= 4,
    "capture buffering must remain bounded to at most 80ms");
_Static_assert(MIC_FRAME_DISPATCH_BLOCKS > 0 && MIC_FRAME_DISPATCH_BLOCKS <= 2,
    "frame dispatch must remain bounded to at most 40ms");
typedef struct {
    uint32_t sequence, generation;
    bool motor_active;
    uint8_t reserved[7]; /* Explicit padding before the two local timestamps. */
    /* Local monotonic instants, never transmitted with the audio packet.
     * captured_us means a FULL raw block returned from DMA, not the PDM
     * sampling instant. enqueued_us precedes the zero-wait queue copy. */
    uint64_t captured_us, enqueued_us;
    _Alignas(16) int16_t pcm[MIC_CAPTURE_FRAMES_PER_PACKET * MIC_CHANNEL_COUNT];
} mic_capture_block_t;
_Static_assert(offsetof(mic_capture_block_t, pcm) % 16 == 0, "capture PCM member alignment");
_Static_assert(sizeof(mic_capture_block_t) % 16 == 0, "capture block stride alignment");
/* Frame callbacks used to invoke the codec/radio sender directly from this
 * 20ms DSP owner.  They now cross this fixed, zero-wait queue.  The worker is
 * allowed to take sender locks and call radio APIs; the PCM owner is not. */
typedef struct {
    mic_audio_frame_cb_t callback;
    void *user_ctx;
    dual_end_audio_analysis_metadata_t analysis;
    uint64_t enqueued_us, captured_us;
    uint32_t boot_id, capture_generation, dispatch_epoch;
    uint16_t spatial_generation;
    uint8_t spatial_mode;
    _Alignas(16) int16_t pcm[MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET];
} mic_frame_dispatch_t;
typedef struct {
    dual_end_audio_owner_timing_t timing;
    uint32_t boot_id, capture_generation, dispatch_epoch;
    uint16_t spatial_generation;
    uint8_t spatial_mode;
    uint32_t work_us;
    uint64_t enqueued_us, dequeued_us, control_ready_us, captured_us, callback_started_us;
} mic_owner_timing_event_t;
_Static_assert(offsetof(mic_frame_dispatch_t, pcm) % 16 == 0,
    "frame-dispatch PCM member alignment");
static QueueHandle_t s_capture_blocks, s_diagnostic_blocks, s_frame_dispatch_blocks,
    s_owner_timing_events;
static SemaphoreHandle_t s_capture_io_lock, s_frame_dispatch_control;
static TaskHandle_t s_capture_task, s_diagnostic_task, s_telemetry_task,
    s_frame_dispatch_task;
static portMUX_TYPE s_pipeline_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_pipeline_generation, s_capture_sequence;
static uint32_t s_dma_overflows;
static volatile bool s_dma_overflow_pending;
static uint32_t s_capture_overloads, s_diagnostic_drops, s_pipeline_highwater;
static uint32_t s_pipeline_deadline_misses, s_diagnostic_frames;
static uint64_t s_diagnostic_total_us;
static uint32_t s_diagnostic_max_us, s_diagnostic_sequence;
static dual_end_audio_analysis_metadata_t s_diagnostic_metadata;
static bool s_analysis_available;
static int64_t s_diagnostic_completed_us;
static uint32_t s_diagnostic_budget_skips, s_diagnostic_overload_skips, s_diagnostic_stale_skips;
static uint64_t s_diagnostic_completed_frames, s_diagnostic_completed_total_us;
static uint32_t s_diagnostic_lifetime_max_us;
static uint64_t s_pipeline_last_overrun_us;
static uint32_t s_owner_timing_event_drops;
static uint32_t s_frame_dispatch_queue_full, s_frame_dispatch_invalidated;
static uint32_t s_frame_dispatch_unavailable, s_frame_dispatch_highwater, s_frame_dispatch_late_events;
/* Only the owner modifies its aggregate. Background gets a complete value
 * under a short copy lock; it cannot delay or reset DSP/encoder histories. */
static mic_pipeline_telemetry_t s_pipeline_window, s_pipeline_report;
static mic_wiener_timing_histogram_t s_pipeline_wiener_histogram[2];
static bool s_pipeline_window_active, s_pipeline_report_ready;
static uint32_t s_pipeline_report_sequence, s_pipeline_windows_lost;
/* A second immutable bundle is intentionally independent from the one-shot
 * console report.  The logger clears only s_pipeline_report_ready; CDC query
 * callers therefore cannot steal, delay, or replay a console telemetry
 * window.  Pipeline and spatial state are captured at the same close boundary
 * so diagnostics cannot join a newer spatial record to an older window. */
static mic_audio_diagnostics_snapshot_t s_pipeline_diagnostics_cache;
static bool s_pipeline_diagnostics_cache_ready;

static void mic_frame_dispatch_reset_stale_queue(void)
{
    if (!s_frame_dispatch_blocks) return;
    const UBaseType_t queued=uxQueueMessagesWaiting(s_frame_dispatch_blocks);
    if (queued==0U || xQueueReset(s_frame_dispatch_blocks)!=pdPASS) return;
    /* Count intentionally discarded old-codec frames in the existing
     * invalidation metric so the transition remains visible in telemetry. */
    portENTER_CRITICAL(&s_pipeline_lock);
    const uint32_t dropped=(uint32_t)queued;
    s_frame_dispatch_invalidated=UINT32_MAX-s_frame_dispatch_invalidated<dropped ?
        UINT32_MAX : s_frame_dispatch_invalidated+dropped;
    portEXIT_CRITICAL(&s_pipeline_lock);
}

static void mic_frame_dispatch_control_lock(void)
{
    /* Before worker creation there is no concurrent callback to serialize.
     * This lock is deliberately never acquired from the regular PCM/DSP path. */
    if (s_frame_dispatch_control)
        (void)xSemaphoreTake(s_frame_dispatch_control, portMAX_DELAY);
}

static void mic_frame_dispatch_control_unlock(void)
{
    if (s_frame_dispatch_control)
        xSemaphoreGive(s_frame_dispatch_control);
}

typedef struct {
    mic_audio_stats_t stats;
    int64_t raw_sum_sq[2], pcm_sum_sq[2];
    uint32_t sequence;
} mic_stats_snapshot_t;
static mic_stats_snapshot_t s_stats_snapshot;

static void mic_pipeline_close_window(uint64_t now_us)
{
    if (!s_pipeline_window_active || !s_pipeline_window.w.n) return;
    /* Capture spatial state at this owner boundary before publishing the
     * pipeline window.  Do not let a later diagnostics query relabel a newer
     * spatial producer sequence as this closed pipeline sequence. */
    dual_end_spatial_telemetry_t spatial_at_close;
    mic_wiener_diagnostics_t wiener_at_close = {0};
    portENTER_CRITICAL(&s_config_lock);
    spatial_at_close=s_spatial_telemetry;
    if ((s_spatial->stats.mode == DUAL_END_SPATIAL_DAS_WIENER ||
         s_spatial->stats.mode == DUAL_END_SPATIAL_WIENER_STEREO) &&
        s_spatial->stats.generation == spatial_at_close.generation) {
        wiener_at_close.available = true;
        wiener_at_close.channels = s_spatial->stats.mode == DUAL_END_SPATIAL_WIENER_STEREO ? 2U : 1U;
        wiener_at_close.noise_valid[0] = s_spatial->wiener.noise_valid;
        wiener_at_close.processed_hops[0] = s_spatial->wiener.processed_hops;
        wiener_at_close.queue_underflows[0] = s_spatial->wiener.queue_underflows;
        wiener_at_close.queue_overflows[0] = s_spatial->wiener.queue_overflows;
        if (wiener_at_close.channels == 2U) {
            wiener_at_close.noise_valid[1] = s_spatial->wiener_right.noise_valid;
            wiener_at_close.processed_hops[1] = s_spatial->wiener_right.processed_hops;
            wiener_at_close.queue_underflows[1] = s_spatial->wiener_right.queue_underflows;
            wiener_at_close.queue_overflows[1] = s_spatial->wiener_right.queue_overflows;
        }
    }
    portEXIT_CRITICAL(&s_config_lock);
    mic_pipeline_telemetry_t snapshot=s_pipeline_window;
    snapshot.sampled_us=now_us;
    for (unsigned channel=0;channel<2U;++channel)
        snapshot.wiener_timing[channel]=
            mic_wiener_timing_histogram_summary(&s_pipeline_wiener_histogram[channel]);
    portENTER_CRITICAL(&s_pipeline_lock);
    /* FFT can complete between the caller's timestamp and this lock. Sample
     * the close instant under the SAME lock as its completion/counters so
     * fft_us can never be in the future of this immutable record. Timer read
     * is constant-time; no formatting/SDK codec/heap work under this lock. */
    const uint64_t sampled_us=(uint64_t)esp_timer_get_time();
    if(sampled_us>snapshot.sampled_us)snapshot.sampled_us=sampled_us;
    if (s_pipeline_report_ready && s_pipeline_windows_lost!=UINT32_MAX) ++s_pipeline_windows_lost;
    /* Zero means "no completed diagnostics cache" to the CDC context guard.
     * Preserve that sentinel even if this long-lived counter wraps. */
    if (++s_pipeline_report_sequence == 0U) ++s_pipeline_report_sequence;
    snapshot.sequence=s_pipeline_report_sequence;snapshot.window=snapshot.sequence;
    snapshot.diag_q=s_diagnostic_drops;snapshot.diag_b=s_diagnostic_budget_skips;
    snapshot.diag_o=s_diagnostic_overload_skips;snapshot.diag_s=s_diagnostic_stale_skips;
    snapshot.fft_us=(uint64_t)s_diagnostic_completed_us;
    snapshot.fft_n=s_diagnostic_completed_frames;snapshot.fft_u=s_diagnostic_completed_total_us;
    snapshot.fft_hi=s_diagnostic_lifetime_max_us;snapshot.win_lost=s_pipeline_windows_lost;
    snapshot.late=s_pipeline_deadline_misses;snapshot.raw_seq=s_capture_sequence;
    snapshot.q_hi_all=s_pipeline_highwater;
    snapshot.dispatch_queue_full=s_frame_dispatch_queue_full;
    snapshot.dispatch_invalidated=s_frame_dispatch_invalidated;
    snapshot.dispatch_unavailable=s_frame_dispatch_unavailable;
    snapshot.dispatch_event_lost=s_owner_timing_event_drops;
    snapshot.dispatch_queue_highwater=s_frame_dispatch_highwater;
    snapshot.dispatch_late_events=s_frame_dispatch_late_events;
    s_pipeline_report=snapshot;s_pipeline_report_ready=true;
    memset(s_pipeline_wiener_histogram,0,sizeof(s_pipeline_wiener_histogram));
    portEXIT_CRITICAL(&s_pipeline_lock);
    s_pipeline_window_active=false;

    const bool spatial_ready=spatial_at_close.boot_id==snapshot.boot_id &&
        spatial_at_close.session_id==snapshot.session_id &&
        spatial_at_close.codec_known==snapshot.codec_known &&
        spatial_at_close.codec==snapshot.codec &&
        spatial_at_close.applied==snapshot.mode &&
        spatial_at_close.generation==snapshot.generation;
    const mic_audio_diagnostics_snapshot_t diagnostics={
        .ready=true,
        .spatial_ready=spatial_ready,
        .cache_sequence=snapshot.sequence,
        .pipeline=snapshot,
        .spatial=spatial_at_close,
        .wiener=wiener_at_close,
    };
    /* Publish the complete immutable bundle in one short copy.  Its spatial
     * sequence remains untouched; cache_sequence is the only valid join key. */
    portENTER_CRITICAL(&s_pipeline_lock);
    s_pipeline_diagnostics_cache=diagnostics;
    s_pipeline_diagnostics_cache_ready=true;
    portEXIT_CRITICAL(&s_pipeline_lock);
}

static void mic_pipeline_record_block(const mic_capture_block_t *block, uint64_t dequeued_us,
    uint32_t dsp_us, uint32_t spatial_us, uint32_t owner_us,
    const mic_capture_health_t *capture_health,
    const dual_end_audio_owner_timing_t *owner_timing,
    const mic_nlms_block_t *nlms_block,
    const dual_end_spatial_wiener_timing_t *wiener_timing)
{
    /* Context was published by the SAME owner after recording this callback;
     * never attach a current session to totals from an earlier session. */
    portENTER_CRITICAL(&s_config_lock);
    dual_end_spatial_telemetry_t identity=s_spatial_telemetry;
    const uint8_t output_gain_l_x=(uint8_t)s_config[2];
    const uint8_t output_gain_r_x=(uint8_t)s_config[3];
    portEXIT_CRITICAL(&s_config_lock);
    if (s_pipeline_window_active &&
        (s_pipeline_window.boot_id!=identity.boot_id || s_pipeline_window.session_id!=identity.session_id ||
         s_pipeline_window.codec!=identity.codec || s_pipeline_window.codec_known!=identity.codec_known ||
         s_pipeline_window.mode!=identity.applied || s_pipeline_window.generation!=identity.generation ||
         s_pipeline_window.capture_generation!=block->generation ||
         s_pipeline_window.output_gain_l_x!=output_gain_l_x ||
         s_pipeline_window.output_gain_r_x!=output_gain_r_x))
        mic_pipeline_close_window(dequeued_us);
    if (!s_pipeline_window_active) {
        memset(&s_pipeline_window,0,sizeof(s_pipeline_window));
        memset(s_pipeline_wiener_histogram,0,sizeof(s_pipeline_wiener_histogram));
        s_pipeline_window.boot_id=identity.boot_id;s_pipeline_window.session_id=identity.session_id;
        s_pipeline_window.codec=identity.codec;s_pipeline_window.codec_known=identity.codec_known;
        s_pipeline_window.mode=(uint8_t)identity.applied;s_pipeline_window.generation=identity.generation;
        s_pipeline_window.output_gain_l_x=output_gain_l_x;
        s_pipeline_window.output_gain_r_x=output_gain_r_x;
        s_pipeline_window.capture_generation=block->generation;
        s_pipeline_window.start_us=dequeued_us;s_pipeline_window_active=true;
    }
    /* Reject malformed/backwards instants or durations not representable in
     * this u32 metric. Their n is lower than w.n, so consumers retain missing
     * queue-age evidence rather than receiving a wrapped low latency. */
    if(dequeued_us>=block->enqueued_us && dequeued_us-block->enqueued_us<=UINT32_MAX)
        mic_pipeline_metric_add(&s_pipeline_window.q,(uint32_t)(dequeued_us-block->enqueued_us));
    if(dequeued_us>=block->captured_us && dequeued_us-block->captured_us<=UINT32_MAX)
        mic_pipeline_metric_add(&s_pipeline_window.a,(uint32_t)(dequeued_us-block->captured_us));
    mic_pipeline_metric_add(&s_pipeline_window.d,dsp_us);
    mic_pipeline_metric_add(&s_pipeline_window.s,spatial_us);
    mic_pipeline_metric_add(&s_pipeline_window.w,owner_us);
    mic_capture_health_merge(&s_pipeline_window.capture_health, capture_health);
    if (owner_timing && owner_timing->valid &&
        owner_timing->session_id == identity.session_id &&
        owner_timing->codec == identity.codec) {
        mic_pipeline_metric_add(&s_pipeline_window.owner_mutex_wait,
                                owner_timing->mutex_wait_us);
        if (owner_timing->encode_attempted)
            mic_pipeline_metric_add(&s_pipeline_window.owner_encode,
                                    owner_timing->encode_us);
        if (owner_timing->radio_submitted)
            mic_pipeline_metric_add(&s_pipeline_window.owner_radio_submit,
                                    owner_timing->radio_submit_us);
    }
    /* Wiener duration was measured by this same DSP owner immediately around
     * each filter call. Add it directly to the matching immutable pipeline
     * window; routing it through the asynchronous sender-completion queue
     * could attach a late callback to the next five-second window. */
    const uint8_t expected_wiener_channels = identity.applied == DUAL_END_SPATIAL_DAS_WIENER ? 1U :
        identity.applied == DUAL_END_SPATIAL_WIENER_STEREO ? 2U : 0U;
    if (wiener_timing && wiener_timing->valid && expected_wiener_channels &&
        wiener_timing->channels == expected_wiener_channels) {
        for (unsigned channel=0;channel<expected_wiener_channels;++channel)
            mic_wiener_timing_histogram_add(&s_pipeline_wiener_histogram[channel],
                                             wiener_timing->us[channel]);
    }
    mic_nlms_telemetry_add(&s_pipeline_window.nlms, nlms_block);
    const uint64_t now_us=(uint64_t)esp_timer_get_time();
    if(now_us-s_pipeline_window.start_us>=MIC_PIPELINE_WINDOW_US)mic_pipeline_close_window(now_us);
}

/* Called only by the microphone DSP owner while it drains the worker's
 * zero-wait completion queue.  The sender worker measures the real codec/radio
 * callback work, but this owner remains the sole writer of MIC_PIPELINE runtime
 * windows and pipeline aggregates.  Mismatched or pre-reset completions are
 * deliberately discarded instead of being joined to a later stream. */
static void mic_pipeline_record_owner_timing_event(const mic_owner_timing_event_t *event)
{
    if (!event) return;
    if (event->dispatch_epoch != mic_frame_dispatch_epoch_load()) {
        portENTER_CRITICAL(&s_pipeline_lock);
        if (s_frame_dispatch_late_events!=UINT32_MAX) ++s_frame_dispatch_late_events;
        portEXIT_CRITICAL(&s_pipeline_lock);
        return;
    }
    if (event->timing.valid)
        dual_end_runtime_record_outcome(DUAL_END_STAGE_MIC_PIPELINE,
            event->timing.codec,event->timing.session_id,event->work_us,
            event->timing.outcome);
    if (!s_pipeline_window_active ||
        s_pipeline_window.boot_id!=event->boot_id ||
        s_pipeline_window.session_id!=event->timing.session_id ||
        !s_pipeline_window.codec_known ||
        s_pipeline_window.codec!=event->timing.codec ||
        s_pipeline_window.mode!=event->spatial_mode ||
        s_pipeline_window.generation!=event->spatial_generation ||
        s_pipeline_window.capture_generation!=event->capture_generation ||
        (event->enqueued_us && event->enqueued_us<s_pipeline_window.start_us)) {
        portENTER_CRITICAL(&s_pipeline_lock);
        if (s_frame_dispatch_late_events!=UINT32_MAX) ++s_frame_dispatch_late_events;
        portEXIT_CRITICAL(&s_pipeline_lock);
        return;
    }
    if (event->enqueued_us && event->dequeued_us>=event->enqueued_us &&
        event->dequeued_us-event->enqueued_us<=UINT32_MAX)
        mic_pipeline_metric_add(&s_pipeline_window.dispatch_queue_wait,
            (uint32_t)(event->dequeued_us-event->enqueued_us));
    if (event->control_ready_us>=event->dequeued_us && event->dequeued_us &&
        event->control_ready_us-event->dequeued_us<=UINT32_MAX)
        mic_pipeline_metric_add(&s_pipeline_window.dispatch_control_wait,
            (uint32_t)(event->control_ready_us-event->dequeued_us));
    if (event->captured_us && event->callback_started_us>=event->captured_us &&
        event->callback_started_us-event->captured_us<=UINT32_MAX)
        mic_pipeline_metric_add(&s_pipeline_window.dispatch_frame_age,
            (uint32_t)(event->callback_started_us-event->captured_us));
    if (event->timing.valid &&
        event->timing.session_id == s_pipeline_window.session_id &&
        event->timing.codec == s_pipeline_window.codec) {
        mic_pipeline_metric_add(&s_pipeline_window.owner_mutex_wait,
                                event->timing.mutex_wait_us);
        if (event->timing.encode_attempted)
            mic_pipeline_metric_add(&s_pipeline_window.owner_encode,
                                    event->timing.encode_us);
        if (event->timing.radio_submitted)
            mic_pipeline_metric_add(&s_pipeline_window.owner_radio_submit,
                                    event->timing.radio_submit_us);
    }
}

static void mic_drain_owner_timing_events(void)
{
    if (!s_owner_timing_events) return;
    mic_owner_timing_event_t event;
    while (xQueueReceive(s_owner_timing_events,&event,0)==pdTRUE)
        mic_pipeline_record_owner_timing_event(&event);
}

static bool mic_diagnostic_sample_due(uint32_t sequence, uint32_t *last_sequence, bool *started)
{
    if (*started && (uint32_t)(sequence - *last_sequence) < MIC_DIAGNOSTIC_SAMPLE_FRAMES) return false;
    *last_sequence = sequence;
    *started = true;
    return true;
}

static void mic_pipeline_reset(void) {
    /* This reset is audio-owner-only. Publish a final partial window before
     * clearing queues; the background task never resets any capture state. */
    mic_pipeline_close_window((uint64_t)esp_timer_get_time());
    /* Suspension/filter reset may discard old PCM, but must not race a worker
     * that has already passed its epoch check.  This is a transition path,
     * not the regular 20 ms DSP loop. */
    mic_frame_dispatch_control_lock();
    portENTER_CRITICAL(&s_pipeline_lock);
    ++s_pipeline_generation;
    mic_frame_dispatch_invalidate();
    memset(&s_diagnostic_metadata,0,sizeof(s_diagnostic_metadata));
    s_diagnostic_completed_us = 0;
    portEXIT_CRITICAL(&s_pipeline_lock);
    if (s_capture_blocks) xQueueReset(s_capture_blocks);
    if (s_diagnostic_blocks) xQueueReset(s_diagnostic_blocks);
    mic_frame_dispatch_control_unlock();
}

/* ISR only records the discontinuity; no logging, allocation or DSP here. */
static bool IRAM_ATTR mic_dma_overflow(i2s_chan_handle_t channel, i2s_event_data_t *event, void *ctx) {
    (void)channel; (void)event; (void)ctx;
    portENTER_CRITICAL_ISR(&s_pipeline_lock);
    ++s_dma_overflows; s_dma_overflow_pending=true;
    portEXIT_CRITICAL_ISR(&s_pipeline_lock);
    return false;
}

static void mic_capture_io_task(void *arg) {
    mic_capture_block_t *block=arg;
    (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    size_t filled=0;
    unsigned empty_reads=0;
    while (true) {
        if (!s_capture_enabled || !s_capture_io_running) { filled=0; empty_reads=0; vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        xSemaphoreTake(s_capture_io_lock,portMAX_DELAY);
        if (!s_capture_enabled || !s_capture_io_running) { xSemaphoreGive(s_capture_io_lock); filled=0; continue; }
        portENTER_CRITICAL(&s_pipeline_lock);
        uint32_t generation=s_pipeline_generation;
        portEXIT_CRITICAL(&s_pipeline_lock);
        if (filled && block->generation!=generation) filled=0;
        block->generation=generation;
        portENTER_CRITICAL(&s_pipeline_lock);
        bool overflow=s_dma_overflow_pending;
        portEXIT_CRITICAL(&s_pipeline_lock);
        if(overflow) {
            s_capture_enabled=false;
            xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);
            xSemaphoreGive(s_capture_io_lock);
            ESP_LOGE(TAG,"DMA overflow: capture stopped; explicit re-enable required");
            continue;
        }
        size_t bytes=0;
        esp_err_t ret=i2s_channel_read(s_mic_rx_chan,(uint8_t*)block->pcm+filled,
            MIC_CAPTURE_BUFFER_BYTES-filled,&bytes,pdMS_TO_TICKS(MIC_READ_TIMEOUT_MS));
        if(bytes)empty_reads=0;
        else if(++empty_reads>=3) {
            s_capture_enabled=false;
            xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);
            ESP_LOGE(TAG,"three empty capture reads: capture stopped");
        }
        /* Timeout may still return a partial read. Preserve those samples. */
        if ((ret==ESP_OK || ret==ESP_ERR_TIMEOUT) &&
            bytes<=MIC_CAPTURE_BUFFER_BYTES-filled &&
            !(bytes % (sizeof(int16_t)*MIC_CHANNEL_COUNT))) filled+=bytes;
        else {
            ESP_LOGE(TAG,"capture IO error=%s bytes=%u; stopping rather than hiding a discontinuity",
                esp_err_to_name(ret),(unsigned)bytes);
            s_capture_enabled=false;
            xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);
        }
        if (filled==MIC_CAPTURE_BUFFER_BYTES) {
            block->captured_us=(uint64_t)esp_timer_get_time();
            block->sequence=++s_capture_sequence;
            block->motor_active=s_motor_active;
            block->enqueued_us=(uint64_t)esp_timer_get_time();
            if (xQueueSend(s_capture_blocks,block,0)!=pdTRUE) {
                portENTER_CRITICAL(&s_pipeline_lock); ++s_capture_overloads; portEXIT_CRITICAL(&s_pipeline_lock);
                /* Persistent compute overload cannot be repaired by buffering.
                 * Fail closed; explicit re-enable drains DMA and resets DSP. */
                s_capture_enabled=false;
                xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);
                ESP_LOGE(TAG,"capture pipeline full at raw frame=%" PRIu32 "; capture stopped",block->sequence);
            } else {
                uint32_t depth=uxQueueMessagesWaiting(s_capture_blocks);
                portENTER_CRITICAL(&s_pipeline_lock);
                if(depth>s_pipeline_highwater)s_pipeline_highwater=depth;
                portEXIT_CRITICAL(&s_pipeline_lock);
            }
            filled=0;
        }
        xSemaphoreGive(s_capture_io_lock);
    }
}

static void mic_diagnostic_task(void *arg) {
    mic_capture_block_t *block=arg;
    uint32_t generation=UINT32_MAX;
    uint32_t previous_sequence=0;
    mic_diagnostic_budget_t budget={0};
    (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    while (true) {
        if(xQueueReceive(s_diagnostic_blocks,block,portMAX_DELAY)!=pdTRUE)continue;
        const uint64_t started_us=(uint64_t)esp_timer_get_time();
        portENTER_CRITICAL(&s_pipeline_lock);
        const bool stale=block->generation!=s_pipeline_generation ||
            started_us<block->captured_us || started_us-block->captured_us>MIC_DIAGNOSTIC_MAX_AGE_US;
        const bool overloaded=s_pipeline_last_overrun_us &&
            (s_pipeline_last_overrun_us>=started_us || started_us-s_pipeline_last_overrun_us<UINT64_C(1000000));
        if(stale && s_diagnostic_stale_skips!=UINT32_MAX)++s_diagnostic_stale_skips;
        else if(overloaded && s_diagnostic_overload_skips!=UINT32_MAX)++s_diagnostic_overload_skips;
        portEXIT_CRITICAL(&s_pipeline_lock);
        if(stale || overloaded)continue;
        if(!mic_diagnostic_budget_admit(&budget,started_us)) {
            portENTER_CRITICAL(&s_pipeline_lock);
            if(s_diagnostic_budget_skips!=UINT32_MAX)++s_diagnostic_budget_skips;
            portEXIT_CRITICAL(&s_pipeline_lock);
            continue;
        }
        /* Planned sampling gaps are not contiguous audio. Weight the EMA by
         * elapsed raw-frame count; reset only for a new capture generation,
         * reordered sequence, or a long gap, not every scheduled 200 ms. */
        const uint32_t gap = block->sequence - previous_sequence;
        if(block->generation!=generation || gap == 0 || gap > MIC_DIAGNOSTIC_MAX_GAP_FRAMES) {
            mic_analysis_reset();
            generation=block->generation;
        }
        mic_analysis_set_frame_gap(gap);
        previous_sequence=block->sequence;
        dual_end_audio_analysis_metadata_t metadata={0};
        uint32_t elapsed=mic_analysis_process_frame(block->pcm,MIC_CAPTURE_FRAMES_PER_PACKET,
            block->motor_active,&metadata);
        const int64_t completed_us=esp_timer_get_time();
        /* Wall elapsed includes preemption. It deliberately overestimates
         * background CPU budget rather than concealing radio competition. */
        uint64_t wall_us=(uint64_t)completed_us-started_us;
        mic_diagnostic_budget_charge(&budget,wall_us>UINT32_MAX ? UINT32_MAX : (uint32_t)wall_us);
        portENTER_CRITICAL(&s_pipeline_lock);
        /* Cost is real even if capture changed while FFT was running. Only
         * metadata/freshness publication is conditional on generation. */
        ++s_diagnostic_completed_frames;s_diagnostic_completed_total_us+=elapsed;
        if(elapsed>s_diagnostic_lifetime_max_us)s_diagnostic_lifetime_max_us=elapsed;
        if(generation==s_pipeline_generation) {
            s_diagnostic_metadata=metadata; s_diagnostic_sequence=block->sequence;
            s_diagnostic_completed_us=completed_us;
            ++s_diagnostic_frames; s_diagnostic_total_us+=elapsed;
            if(elapsed>s_diagnostic_max_us)s_diagnostic_max_us=elapsed;
        } else if(s_diagnostic_stale_skips!=UINT32_MAX) {
            ++s_diagnostic_stale_skips;
        }
        portEXIT_CRITICAL(&s_pipeline_lock);
    }
}

/* This task is the only place that invokes the registered frame callback.
 * It may take the sender mutex, encode, submit to radio and emit sender logs.
 * The PCM/DSP owner below only makes a bounded zero-wait copy into its queue. */
static void mic_frame_dispatch_task_fn(void *arg)
{
    (void)arg;
    mic_frame_dispatch_t frame;
    while (true) {
        if (xQueueReceive(s_frame_dispatch_blocks,&frame,portMAX_DELAY)!=pdTRUE)
            continue;
        const uint64_t dequeued_us=(uint64_t)esp_timer_get_time();
        /* A configuration transition holds this lock across sender session
         * mutation.  Taking it here prevents a queued old frame from being
         * encoded under a new codec/layout/session.  This worker is outside
         * the PCM/DSP real-time owner, where no blocking sender path remains. */
        if (!s_frame_dispatch_control ||
            xSemaphoreTake(s_frame_dispatch_control,portMAX_DELAY)!=pdTRUE) {
            portENTER_CRITICAL(&s_pipeline_lock);
            if (s_frame_dispatch_unavailable!=UINT32_MAX) ++s_frame_dispatch_unavailable;
            portEXIT_CRITICAL(&s_pipeline_lock);
            continue;
        }
        const uint64_t control_ready_us=(uint64_t)esp_timer_get_time();
        if (!s_capture_enabled || frame.dispatch_epoch!=mic_frame_dispatch_epoch_load()) {
            portENTER_CRITICAL(&s_pipeline_lock);
            if (s_frame_dispatch_invalidated!=UINT32_MAX) ++s_frame_dispatch_invalidated;
            portEXIT_CRITICAL(&s_pipeline_lock);
            xSemaphoreGive(s_frame_dispatch_control);
            continue;
        }
        const int64_t work_started_us=esp_timer_get_time();
        if (frame.callback) {
            frame.callback(frame.pcm,MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET,
                           &frame.analysis,frame.user_ctx);
        }
        const int64_t work_finished_us=esp_timer_get_time();
        dual_end_audio_owner_timing_t timing={0};
        const bool have_timing=dual_end_audio_sender_take_owner_timing(&timing);
        xSemaphoreGive(s_frame_dispatch_control);
        if (!have_timing) continue;
        const int64_t elapsed_us=work_finished_us-work_started_us;
        const mic_owner_timing_event_t event={
            .timing=timing,
            .enqueued_us=frame.enqueued_us,
            .dequeued_us=dequeued_us,
            .control_ready_us=control_ready_us,
            .captured_us=frame.captured_us,
            .callback_started_us=(uint64_t)work_started_us,
            .boot_id=frame.boot_id,
            .capture_generation=frame.capture_generation,
            .dispatch_epoch=frame.dispatch_epoch,
            .spatial_generation=frame.spatial_generation,
            .spatial_mode=frame.spatial_mode,
            .work_us=elapsed_us<=0 ? 0U :
                ((uint64_t)elapsed_us>UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us),
        };
        if (!s_owner_timing_events ||
            xQueueSend(s_owner_timing_events,&event,0)!=pdTRUE) {
            /* This is optional evidence only.  It must never delay audio or
             * cause a callback retry; the real sender result is already final. */
            portENTER_CRITICAL(&s_pipeline_lock);
            if (s_owner_timing_event_drops!=UINT32_MAX) ++s_owner_timing_event_drops;
            portEXIT_CRITICAL(&s_pipeline_lock);
        }
    }
}

static void mic_telemetry_task(void *arg)
{
    (void)arg;(void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    TickType_t last_spatial=0,last_analysis=0;
    uint32_t spatial_seq=0,stats_seq=0;
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        TickType_t now=xTaskGetTickCount();
        /* Formatting happens here even if FFT is starved, budget-skipped,
         * unavailable or permanently disabled. No encoder or DMA locks. */
        /* Emit the legacy spatial snapshot at the same five-second cadence as
         * the immutable windows below. This keeps USB/logger pressure bounded
         * instead of layering new health parts onto a one-second burst. */
        if(now-last_spatial>=pdMS_TO_TICKS(5000)) {
            portENTER_CRITICAL(&s_pipeline_lock);
            const uint32_t overloads=s_capture_overloads,drops=s_diagnostic_drops;
            const uint32_t highwater=s_pipeline_highwater,misses=s_pipeline_deadline_misses;
            const uint32_t dma_overflows=s_dma_overflows,diag_sequence=s_diagnostic_sequence;
            const int64_t completed_us=s_diagnostic_completed_us;
            portEXIT_CRITICAL(&s_pipeline_lock);
            const int64_t age_us=completed_us ? esp_timer_get_time()-completed_us : -1;
            /* Preserve the older text IO observation for archived collectors.
             * New strict attribution uses audio.pipeline + runtime IO parts. */
            ESP_LOGI(TAG,"pipeline diag_raw_seq=%" PRIu32 " queue_highwater=%" PRIu32
                " overload_stops=%" PRIu32 " diag_drops=%" PRIu32 " work_over_20ms=%" PRIu32 " dma_overflows=%" PRIu32
                " diag_age_ms=%" PRId64 " diag_interval_ms=200",
                diag_sequence,highwater,overloads,drops,misses,dma_overflows,age_us<0 ? -1 : age_us/1000);
            portENTER_CRITICAL(&s_config_lock);
            dual_end_spatial_telemetry_t spatial=s_spatial_telemetry;
            portEXIT_CRITICAL(&s_config_lock);
            spatial.sequence=++spatial_seq;
            char line[DUAL_END_TELEMETRY_JSON_BYTES];
            for(unsigned part=0;part<DUAL_END_SPATIAL_TELEMETRY_PARTS;++part) {
                if(dual_end_spatial_telemetry_format(&spatial,part,line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
                vTaskDelay(1); /* Bound the log burst; yield to radio/USB. */
            }
            last_spatial=now;
        }
        portENTER_CRITICAL(&s_pipeline_lock);
        const bool report_ready=s_pipeline_report_ready;
        mic_pipeline_telemetry_t pipeline=s_pipeline_report;
        s_pipeline_report_ready=false;
        mic_stats_snapshot_t cached=s_stats_snapshot;
        portEXIT_CRITICAL(&s_pipeline_lock);
        if(report_ready) {
            char line[MIC_PIPELINE_TELEMETRY_BYTES];
            for(unsigned part=0;part<MIC_PIPELINE_TELEMETRY_PARTS;++part) {
                if(mic_pipeline_telemetry_format(&pipeline,part,line,sizeof(line)))ESP_LOGI(TAG,"%s",line);
                vTaskDelay(1);
            }
            char observability[MIC_OBSERVABILITY_TELEMETRY_BYTES];
            for(unsigned part=0;part<MIC_CAPTURE_HEALTH_TELEMETRY_PARTS;++part) {
                if(mic_capture_health_telemetry_format(&pipeline,part,observability,sizeof(observability)))
                    ESP_LOGI(TAG,"%s",observability);
                vTaskDelay(1);
            }
            for(unsigned part=0;part<MIC_OWNER_TIMING_TELEMETRY_PARTS;++part) {
                if(mic_owner_timing_telemetry_format(&pipeline,part,observability,sizeof(observability)))
                    ESP_LOGI(TAG,"%s",observability);
                vTaskDelay(1);
            }
            for(unsigned part=0;part<MIC_DISPATCH_TELEMETRY_PARTS;++part) {
                if(mic_dispatch_telemetry_format(&pipeline,part,observability,sizeof(observability)))
                    ESP_LOGI(TAG,"%s",observability);
                vTaskDelay(1);
            }
            for(unsigned part=0;part<MIC_OUTPUT_HEALTH_TELEMETRY_PARTS;++part) {
                if(mic_output_health_telemetry_format(&pipeline,part,observability,sizeof(observability)))
                    ESP_LOGI(TAG,"%s",observability);
                vTaskDelay(1);
            }
            if (pipeline.nlms.blocks) {
                for(unsigned part=0;part<MIC_NLMS_TELEMETRY_PARTS;++part) {
                    if(mic_nlms_telemetry_format(&pipeline,part,observability,sizeof(observability)))
                        ESP_LOGI(TAG,"%s",observability);
                    vTaskDelay(1);
                }
                char nlms_presence[MIC_NLMS_PRESENCE_TELEMETRY_BYTES];
                for(unsigned part=0;part<MIC_NLMS_PRESENCE_TELEMETRY_PARTS;++part) {
                    if(mic_nlms_presence_telemetry_format(&pipeline,part,nlms_presence,
                                                          sizeof(nlms_presence)))
                        ESP_LOGI(TAG,"%s",nlms_presence);
                    vTaskDelay(1);
                }
            }
        }
        if(cached.sequence==stats_seq)continue;
        stats_seq=cached.sequence;
        mic_audio_stats_t *stats=&cached.stats;
        /* Only the diagnostic task executes double sqrt, callbacks or
         * printf-style formatting. The owner publishes sums by value. */
        if(stats->input_frames) {
            stats->left_raw_rms=(uint32_t)(sqrt((double)cached.raw_sum_sq[0]/stats->input_frames)+0.5);
            stats->right_raw_rms=(uint32_t)(sqrt((double)cached.raw_sum_sq[1]/stats->input_frames)+0.5);
        }
        if(stats->output_frames) {
            stats->left_pcm_rms=(uint32_t)(sqrt((double)cached.pcm_sum_sq[0]/stats->output_frames)+0.5);
            stats->right_pcm_rms=(uint32_t)(sqrt((double)cached.pcm_sum_sq[1]/stats->output_frames)+0.5);
        }
        if(s_stats_callback)s_stats_callback(stats,s_stats_user_ctx);
        if(now-last_analysis>=pdMS_TO_TICKS(MIC_ANALYSIS_RUNTIME_LOG_INTERVAL_MS)) {
            ESP_LOGI(TAG,"analysis rate=%" PRIu32 "fps avg/max=%" PRIu32 "/%" PRIu32
                "us flags=0x%02x tdoa=%d/256 conf/coh=%u/%u hf/motor=%u/%u clip=%u/%u vad=%u gate_open=%u gain=%u/%u",
                stats->analysis_frames,stats->analysis_average_us,stats->analysis_max_us,
                stats->latest_analysis.flags,stats->latest_analysis.tdoa_samples_q8_8,
                stats->latest_analysis.confidence_q15,stats->latest_analysis.coherence_q15,
                stats->latest_analysis.high_frequency_q15,stats->latest_analysis.motor_q15,
                stats->latest_analysis.clip_left_count,stats->latest_analysis.clip_right_count,
                (unsigned)stats->vad_enabled,(unsigned)stats->vad_open,
                (unsigned)stats->output_gain_x,(unsigned)stats->right_output_gain_x);
            last_analysis=now;
        }
    }
}

static void mic_filter_reset(void)
{
    memset(s_hpf_previous_input, 0, sizeof(s_hpf_previous_input));
    memset(s_hpf_previous_output, 0, sizeof(s_hpf_previous_output));
    memset(s_fir_history, 0, sizeof(s_fir_history));
    s_fir_write_index = 0;
    s_decimation_phase = 0;
    s_soft_limited_frames = 0;
    s_soft_limit_min_gain = 1.0f;
    mic_pipeline_reset();
}

static float mic_shared_soft_limit_gain(float stereo_peak)
{
    if (!s_config[8] || stereo_peak <= (float)s_config[9]) {
        return 1.0f;
    }

    const float soft_range = (float)(s_config[10] - s_config[9]);
    const float excess = stereo_peak - (float)s_config[9];
    const float limited_peak = (float)s_config[9] +
                               (soft_range * excess) / (soft_range + excess);
    return limited_peak / stereo_peak;
}

static bool mic_filter_and_decimate(int16_t raw_left,
                                    int16_t raw_right,
                                    int16_t *out_left,
                                    int16_t *out_right)
{
    const int16_t raw[MIC_CHANNEL_COUNT] = {raw_left, raw_right};

    for (size_t channel = 0; channel < MIC_CHANNEL_COUNT; channel++) {
        const float input = (float)raw[channel];
        const float hpf = s_runtime_hpf *
                          (s_hpf_previous_output[channel] +
                           input - s_hpf_previous_input[channel]);
        s_hpf_previous_input[channel] = input;
        s_hpf_previous_output[channel] = hpf;
        s_fir_history[channel][s_fir_write_index] = s_config[4] ? hpf : input;
    }

    if (++s_fir_write_index == MIC_DECIMATOR_FIR_TAPS) s_fir_write_index = 0;
    s_decimation_phase++;
    if (s_decimation_phase < MIC_DECIMATION_FACTOR) {
        return false;
    }
    s_decimation_phase = 0;

    float amplified[MIC_CHANNEL_COUNT];
    for (size_t channel = 0; channel < MIC_CHANNEL_COUNT; channel++) {
        float accumulator = 0.0f;
        size_t history_index = s_fir_write_index ? s_fir_write_index - 1U : MIC_DECIMATOR_FIR_TAPS - 1U;
        if (s_config[6]) {
            /* Two contiguous descending spans, identical coefficient/MAC order.
             * Storage has a 96-float aligned stride, but the logical ring is 95.
             * Never visit the pad or regroup floating-point operations. */
            const size_t first_span = history_index + 1U;
            size_t tap = 0;
            for (; tap < first_span; ++tap) {
                accumulator += s_runtime_fir[tap] * s_fir_history[channel][history_index - tap];
            }
            for (size_t index = MIC_DECIMATOR_FIR_TAPS; tap < MIC_DECIMATOR_FIR_TAPS; ++tap) {
                accumulator += s_runtime_fir[tap] * s_fir_history[channel][--index];
            }
        } else {
            /* The bypass consumes this same newest history sample; no FIR MACs. */
            accumulator = s_fir_history[channel][history_index];
        }
        /* Fixed digital gain scales speech and existing noise together. It is
         * not AGC or a signal-to-noise improvement; keep L/R gains separate. */
        amplified[channel] = accumulator * (float)s_config[2 + channel];
    }

    const float stereo_peak = fmaxf(fabsf(amplified[0]), fabsf(amplified[1]));
    const float soft_limit_gain = mic_shared_soft_limit_gain(stereo_peak);
    if (soft_limit_gain < 1.0f) {
        s_soft_limited_frames++;
        if (soft_limit_gain < s_soft_limit_min_gain) {
            s_soft_limit_min_gain = soft_limit_gain;
        }
    }

    *out_left = clamp_i16_from_float(amplified[0] * soft_limit_gain);
    *out_right = clamp_i16_from_float(amplified[1] * soft_limit_gain);
    return true;
}

static esp_err_t mic_pdm_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = MIC_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = MIC_I2S_DMA_FRAME_NUM;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_mic_rx_chan),
                        TAG,
                        "i2s new channel failed");

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_CAPTURE_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = MIC_PDM_CLK_GPIO,
            .din = MIC_PDM_DATA_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    /*
     * ESP32-S3 does not expose an APLL I2S clock source.  Use its supported
     * 240 MHz PLL for this clock diagnostic: 240 MHz / (9 + 49/64) / 8 gives
     * an average PDM clock of exactly 3.072 MHz with finer edge quantisation
     * than the default 160 MHz PLL source.
     */
    pdm_cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_240M;
#if SOC_I2S_SUPPORTS_PDM_RX_HP_FILTER
    /* The explicit software HPF below owns the 80 Hz response. */
    pdm_cfg.slot_cfg.hp_en = false;
#endif
    pdm_cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_BOTH;

    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_mic_rx_chan, &pdm_cfg),
                        TAG,
                        "i2s PDM RX init failed");
    const i2s_event_callbacks_t callbacks={.on_recv_q_ovf=mic_dma_overflow};
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_mic_rx_chan,&callbacks,NULL),TAG,"DMA overflow callback");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_mic_rx_chan), TAG, "PDM RX enable failed");

    ESP_LOGI(TAG,
             "PDM stereo ready: capture=%dHz/16bit/2ch clock=PLL_240M MCLK/PDM_CLK=%dHz(GPIO%d) "
             "DATA=GPIO%d (no BCLK/WS/RST)",
             MIC_CAPTURE_SAMPLE_RATE_HZ,
             MIC_PDM_CLK_HZ,
             MIC_PDM_CLK_GPIO,
             MIC_PDM_DATA_GPIO);
    ESP_LOGI(TAG,
             "DSP ready: HPF=%dHz FIR-LPF=%dHz/%d taps decimation=%d:1 output=%dHz stereo",
             MIC_HPF_CUTOFF_HZ,
             MIC_DECIMATOR_CUTOFF_HZ,
             MIC_DECIMATOR_FIR_TAPS,
             MIC_DECIMATION_FACTOR,
             MIC_OUTPUT_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG,
             "audio profile=%s fixed_gain=%ux(%.2fdB) default=%dx shared_soft_limit=%d..%d no_release_tail",
             MIC_AUDIO_PROFILE,
             (unsigned)s_output_gain_x,
             (double)(20.0f * log10f((float)s_output_gain_x)),
             CONFIG_MIC_OUTPUT_GAIN_X,
             MIC_SOFT_LIMIT_KNEE,
             MIC_SOFT_LIMIT_CEILING);
    ESP_LOGI(TAG,
             "48k analysis: fixed_calibration=disabled GCC-PHAT=1024"
             " sampled 5Hz available=%u; audio-packet analysis unavailable",
             (unsigned)s_analysis_available);
    return ESP_OK;
}

static esp_err_t mic_warmup_discard(void)
{
    for (int i = 0; i < 8; i++) {
        size_t bytes_read = 0;
        esp_err_t ret=i2s_channel_read(s_mic_rx_chan,
                               s_warmup_discard_buffer,
                               sizeof(s_warmup_discard_buffer),
                               &bytes_read,
                               pdMS_TO_TICKS(100));
        if(ret!=ESP_OK || bytes_read!=sizeof(s_warmup_discard_buffer))return ret==ESP_OK?ESP_FAIL:ret;
    }
    portENTER_CRITICAL(&s_pipeline_lock);s_dma_overflow_pending=false;portEXIT_CRITICAL(&s_pipeline_lock);
    return ESP_OK;
}

static esp_err_t mic_hardware_suspend(void)
{
    esp_err_t ret=i2s_channel_disable(s_mic_rx_chan);
    if(ret!=ESP_OK && ret!=ESP_ERR_INVALID_STATE)return ret;
    /* Already disabled after failed resume is a valid retry boundary. */
    ESP_LOGI(TAG, "suspended: PDM RX disabled and GPIO%d clock stopped", MIC_PDM_CLK_GPIO);
    return ESP_OK;
}

static esp_err_t mic_hardware_resume(void)
{
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_mic_rx_chan), TAG, "PDM RX re-enable failed");

    vTaskDelay(pdMS_TO_TICKS(MIC_RESUME_SETTLE_MS));
    ESP_RETURN_ON_ERROR(mic_warmup_discard(),TAG,"PDM warmup incomplete");
    mic_filter_reset();
    mic_spatial_reset_state_owner(s_spatial->stats.mode);
    ESP_LOGI(TAG, "resumed: PDM clock restored on GPIO%d, DSP state reset",
             MIC_PDM_CLK_GPIO);
    return ESP_OK;
}

/* Publish ACTIVE only for a successful, still-requested physical resume.
 * A stop requested during warmup must never be acknowledged as active. */
static void mic_publish_resume_result(esp_err_t result) {
    if(result==ESP_OK && s_capture_enabled && s_capture_io_running) {
        xEventGroupSetBits(s_mic_state_events,MIC_STATE_ACTIVE_BIT);
    } else {
        xEventGroupClearBits(s_mic_state_events,MIC_STATE_ACTIVE_BIT);
        if(result!=ESP_OK) {
            s_capture_enabled=false; s_capture_io_running=false;
            xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);
        }
    }
}

static void mic_audio_task(void *arg)
{
    (void)arg;
    /* WithCaps can schedule this core before its wrapper publishes the handle.
     * Starter sends the first notification only after handle publication. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    mic_capture_block_t *processing_block = heap_caps_aligned_alloc(16, sizeof(*processing_block), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    int16_t *raw_buf = processing_block ? processing_block->pcm : NULL;
    mic_capture_block_t *capture_block = heap_caps_aligned_alloc(16, sizeof(*capture_block), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    mic_capture_block_t *diagnostic_block = s_analysis_available ?
        heap_caps_aligned_alloc(16, sizeof(*diagnostic_block), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : NULL;
    int16_t *pcm_buf = heap_caps_aligned_alloc(16,
        MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t diagnostic_last_sequence=0, diagnostic_generation=UINT32_MAX;
    bool diagnostic_sample_started=false;
    size_t pcm_frames = 0;
    uint32_t pcm_dispatch_epoch = 0;
    bool pcm_dispatch_allowed = false;
    int64_t frame_processing_us = 0;
    TickType_t last_stats_tick = 0;
    uint32_t produced_packets = 0;
    uint32_t produced_sample_values = 0;
    uint32_t last_produced_packets = 0;
    uint32_t last_produced_sample_values = 0;
    int64_t raw_sum_sq[MIC_CHANNEL_COUNT] = {0};
    int64_t pcm_sum_sq[MIC_CHANNEL_COUNT] = {0};
    int32_t raw_peak[MIC_CHANNEL_COUNT] = {0};
    int32_t pcm_peak[MIC_CHANNEL_COUNT] = {0};
    uint32_t stats_input_frames = 0;
    uint32_t stats_output_frames = 0;
    uint32_t stats_bytes_read = 0;
    uint32_t analysis_frames = 0;
    uint64_t analysis_sum_us = 0;
    uint32_t analysis_max_us = 0;
    dual_end_audio_analysis_metadata_t latest_analysis = {0};

    s_capture_blocks=xQueueCreate(MIC_PIPELINE_BLOCKS,sizeof(mic_capture_block_t));
    s_diagnostic_blocks=diagnostic_block ? xQueueCreate(1,sizeof(mic_capture_block_t)) : NULL;
    s_frame_dispatch_blocks=xQueueCreate(MIC_FRAME_DISPATCH_BLOCKS,sizeof(mic_frame_dispatch_t));
    /* Completion evidence is optional.  Audio sending remains available when
     * this small diagnostics-only queue cannot be allocated. */
    s_owner_timing_events=xQueueCreate(MIC_FRAME_DISPATCH_BLOCKS,
                                       sizeof(mic_owner_timing_event_t));
    s_capture_io_lock=xSemaphoreCreateMutex();
    /* Mandatory only because it preserves the existing atomic callback/session
     * transition contract.  It is not used by the per-frame DSP owner. */
    s_frame_dispatch_control=xSemaphoreCreateMutex();
    if (!raw_buf || !capture_block || !pcm_buf ||
        !s_capture_blocks || !s_frame_dispatch_blocks || !s_capture_io_lock ||
        !s_frame_dispatch_control) {
        ESP_LOGE(TAG, "failed to allocate dual-mic audio buffers");
        free(processing_block); free(capture_block); free(diagnostic_block);
        if(s_capture_blocks)vQueueDelete(s_capture_blocks);
        if(s_diagnostic_blocks)vQueueDelete(s_diagnostic_blocks);
        if(s_frame_dispatch_blocks)vQueueDelete(s_frame_dispatch_blocks);
        if(s_owner_timing_events)vQueueDelete(s_owner_timing_events);
        if(s_capture_io_lock)vSemaphoreDelete(s_capture_io_lock);
        if(s_frame_dispatch_control)vSemaphoreDelete(s_frame_dispatch_control);
        free(pcm_buf);
        xEventGroupSetBits(s_mic_state_events, MIC_START_FAILED_BIT);
        /* Starter owns deletion with matching allocation capabilities. No
         * self-delete cleanup task is allocated when internal RAM is tight. */
        for (;;) (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    /* Core1 retains the DSP owner and higher-priority DMA reader. Codec/radio
     * admission runs on the bounded worker below the DSP priority; CPU-only
     * diagnostics remain on Core0 below wireless/USB. DMA buffers and IO
     * affinity stay unchanged. */
    BaseType_t capture_ok=xTaskCreatePinnedToCore(mic_capture_io_task,"mic_capture_io",
        MIC_CAPTURE_TASK_STACK,capture_block,6,&s_capture_task,DUAL_END_AUDIO_TASK_CORE);
    /* The transport callback is deliberately below the DSP owner priority.
     * If it cannot keep up, the bounded queue drops a frame rather than making
     * the PDM/DSP owner wait behind a sender mutex or radio operation. */
#if MIC_TASK_PSRAM_STACK
    BaseType_t frame_dispatch_ok=capture_ok==pdPASS ? xTaskCreatePinnedToCoreWithCaps(
        mic_frame_dispatch_task_fn,"mic_frame_dispatch",MIC_FRAME_DISPATCH_TASK_STACK,NULL,
        MIC_FRAME_DISPATCH_TASK_PRIORITY,&s_frame_dispatch_task,DUAL_END_AUDIO_TASK_CORE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : pdFAIL;
#else
    BaseType_t frame_dispatch_ok=capture_ok==pdPASS ? xTaskCreatePinnedToCore(
        mic_frame_dispatch_task_fn,"mic_frame_dispatch",MIC_FRAME_DISPATCH_TASK_STACK,NULL,
        MIC_FRAME_DISPATCH_TASK_PRIORITY,&s_frame_dispatch_task,DUAL_END_AUDIO_TASK_CORE) : pdFAIL;
#endif
    /* Codec/layout transition may wait on sender preparation, so keep it off
     * the Core1 PCM owner.  This is mandatory: accepting a runtime codec or
     * spatial setting without a safe executor would make the command outcome
     * ambiguous.  It does no work until a control request notifies it. */
    BaseType_t config_ok=frame_dispatch_ok==pdPASS ? xTaskCreatePinnedToCore(
        mic_config_transition_task_fn,"mic_audio_cfg",MIC_CONFIG_TASK_STACK,NULL,
        MIC_BACKGROUND_TASK_PRIORITY,&s_config_task_handle,MIC_BACKGROUND_TASK_CORE) : pdFAIL;
    /* FFT diagnostics are CPU-only and deferred; preserve internal RAM for
     * PDM DMA and wireless TX. Do not move the capture IO task or DMA buffers. */
#if MIC_TASK_PSRAM_STACK
    BaseType_t diagnostic_ok=capture_ok==pdPASS && frame_dispatch_ok==pdPASS && config_ok==pdPASS && s_diagnostic_blocks ? xTaskCreatePinnedToCoreWithCaps(
        mic_diagnostic_task,"mic_diagnostic",MIC_DIAGNOSTIC_TASK_STACK,
        diagnostic_block,MIC_BACKGROUND_TASK_PRIORITY,&s_diagnostic_task,MIC_BACKGROUND_TASK_CORE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : pdFAIL;
#else
    BaseType_t diagnostic_ok=capture_ok==pdPASS && frame_dispatch_ok==pdPASS && config_ok==pdPASS && s_diagnostic_blocks ? xTaskCreatePinnedToCore(mic_diagnostic_task,
        "mic_diagnostic",MIC_DIAGNOSTIC_TASK_STACK,diagnostic_block,MIC_BACKGROUND_TASK_PRIORITY,
        &s_diagnostic_task,MIC_BACKGROUND_TASK_CORE) : pdFAIL;
#endif
    /* Independent of FFT allocation/admission. A telemetry failure disables
     * optional diagnostics only, never capture or the encoder owner. */
#if MIC_TASK_PSRAM_STACK
    BaseType_t telemetry_ok=capture_ok==pdPASS && frame_dispatch_ok==pdPASS && config_ok==pdPASS ? xTaskCreatePinnedToCoreWithCaps(
        mic_telemetry_task,"mic_telemetry",MIC_TELEMETRY_TASK_STACK,NULL,
        MIC_BACKGROUND_TASK_PRIORITY,&s_telemetry_task,MIC_BACKGROUND_TASK_CORE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : pdFAIL;
#else
    BaseType_t telemetry_ok=capture_ok==pdPASS && frame_dispatch_ok==pdPASS && config_ok==pdPASS ? xTaskCreatePinnedToCore(
        mic_telemetry_task,"mic_telemetry",MIC_TELEMETRY_TASK_STACK,NULL,
        MIC_BACKGROUND_TASK_PRIORITY,&s_telemetry_task,MIC_BACKGROUND_TASK_CORE) : pdFAIL;
#endif
    if(capture_ok!=pdPASS || frame_dispatch_ok!=pdPASS || config_ok!=pdPASS) {
        if(s_capture_task)vTaskDelete(s_capture_task);
        if(s_config_task_handle)vTaskDelete(s_config_task_handle);
#if MIC_TASK_PSRAM_STACK
        if(s_telemetry_task)vTaskDeleteWithCaps(s_telemetry_task);
#else
        if(s_telemetry_task)vTaskDelete(s_telemetry_task);
#endif
#if MIC_TASK_PSRAM_STACK
        if(s_frame_dispatch_task)vTaskDeleteWithCaps(s_frame_dispatch_task);
#else
        if(s_frame_dispatch_task)vTaskDelete(s_frame_dispatch_task);
#endif
#if MIC_TASK_PSRAM_STACK
        if(s_diagnostic_task)vTaskDeleteWithCaps(s_diagnostic_task);
#else
        if(s_diagnostic_task)vTaskDelete(s_diagnostic_task);
#endif
        s_capture_task=s_diagnostic_task=s_telemetry_task=s_frame_dispatch_task=s_config_task_handle=NULL;
        free(processing_block);free(capture_block);free(diagnostic_block);
        free(pcm_buf);
        vQueueDelete(s_capture_blocks);if(s_diagnostic_blocks)vQueueDelete(s_diagnostic_blocks);
        vQueueDelete(s_frame_dispatch_blocks);if(s_owner_timing_events)vQueueDelete(s_owner_timing_events);
        vSemaphoreDelete(s_capture_io_lock);vSemaphoreDelete(s_frame_dispatch_control);
        xEventGroupSetBits(s_mic_state_events,MIC_START_FAILED_BIT);
        for(;;)(void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    }
    if(diagnostic_ok!=pdPASS) {
        /* Optional diagnostics must not turn a resource shortage into no audio. */
        if(s_diagnostic_blocks)vQueueDelete(s_diagnostic_blocks);
        s_diagnostic_blocks=NULL;
        free(diagnostic_block);
        mic_analysis_release();
        s_analysis_available=false;
        ESP_LOGW(TAG,"FFT diagnostics unavailable; capture/encoding remain enabled");
    }
    if(telemetry_ok!=pdPASS)ESP_LOGW(TAG,"optional microphone telemetry unavailable; audio remains enabled");
    if(!s_owner_timing_events)ESP_LOGW(TAG,"optional sender timing telemetry unavailable; audio remains enabled");
    s_capture_io_running=true;
    xTaskNotifyGive(s_capture_task);
    if(s_diagnostic_task)xTaskNotifyGive(s_diagnostic_task);
    if(s_telemetry_task)xTaskNotifyGive(s_telemetry_task);
    xEventGroupSetBits(s_mic_state_events, MIC_START_READY_BIT);

    while (true) {
        if (!s_capture_enabled || !s_capture_io_running) {
            xSemaphoreTake(s_capture_io_lock,portMAX_DELAY);
            s_capture_io_running=false;
            s_state_change_result = mic_hardware_suspend();
            mic_pipeline_reset();
            xSemaphoreGive(s_capture_io_lock);
            xEventGroupClearBits(s_mic_state_events, MIC_STATE_ACTIVE_BIT);
            xEventGroupSetBits(s_mic_state_events, MIC_STATE_SUSPENDED_BIT);
            if(s_state_change_result!=ESP_OK)xEventGroupSetBits(s_mic_state_events,MIC_STATE_FAULT_BIT);

            while (!s_capture_enabled) {
                /* The owner can finish local settings/commits while capture is
                 * suspended, but transport work remains on mic_audio_cfg. */
                mic_owner_apply_dsp_config();
                mic_owner_apply_spatial_commit();
                (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
            }

            if (s_state_change_result == ESP_OK) {
                xEventGroupClearBits(s_mic_state_events,MIC_STATE_SUSPENDED_BIT);
                xSemaphoreTake(s_capture_io_lock,portMAX_DELAY);
                s_state_change_result = mic_hardware_resume();
                s_capture_io_running=s_state_change_result==ESP_OK;
                xSemaphoreGive(s_capture_io_lock);
            }
            pcm_frames = 0;
            frame_processing_us = 0;
            xEventGroupClearBits(s_mic_state_events, MIC_STATE_SUSPENDED_BIT);
            mic_publish_resume_result(s_state_change_result);
            continue;
        }

        size_t bytes_read = 0;
        /* Only the IO task touches DMA while active. This owner never waits
         * for diagnostics or sender locks; the bounded worker invokes the
         * encoder callback after a zero-wait PCM handoff. */
        if(xQueueReceive(s_capture_blocks,processing_block,pdMS_TO_TICKS(MIC_READ_TIMEOUT_MS))!=pdTRUE)continue;
        if(!s_capture_enabled)continue;
        const uint64_t dequeued_us=(uint64_t)esp_timer_get_time();
        bytes_read=MIC_CAPTURE_BUFFER_BYTES;
        if(processing_block->generation != diagnostic_generation) {
            diagnostic_generation=processing_block->generation;
            diagnostic_sample_started=false;
        }
        if(s_diagnostic_blocks && mic_diagnostic_sample_due(processing_block->sequence,
            &diagnostic_last_sequence,&diagnostic_sample_started) &&
            xQueueSend(s_diagnostic_blocks,processing_block,0)!=pdTRUE) {
            /* Count only attempted diagnostic samples, not intentional skips. */
            portENTER_CRITICAL(&s_pipeline_lock);++s_diagnostic_drops;portEXIT_CRITICAL(&s_pipeline_lock);
        }

        size_t sample_values = bytes_read / sizeof(int16_t);
        sample_values -= sample_values % MIC_CHANNEL_COUNT;
        size_t input_frames = sample_values / MIC_CHANNEL_COUNT;

        stats_input_frames += (uint32_t)input_frames;
        stats_bytes_read += (uint32_t)bytes_read;
        /* Time only active chunks after I2S has returned. Partial reads add
         * their work to the same PCM frame without counting blocked I2S time. */
        int64_t processing_started_us = esp_timer_get_time();
        mic_capture_health_t capture_health = {0};
        const uint32_t limited_before=s_soft_limited_frames;

        for (size_t frame = 0; frame < input_frames; frame++) {
            const int16_t raw_left = raw_buf[frame * MIC_CHANNEL_COUNT];
            const int16_t raw_right = raw_buf[frame * MIC_CHANNEL_COUNT + 1U];
            mic_capture_health_record(&capture_health, raw_left, raw_right);


            int16_t pcm_left = 0;
            int16_t pcm_right = 0;
            /* Apply owner-local settings only at a 20 ms output-frame boundary,
             * so one encoded frame never contains a mid-frame gain step.  The
             * config worker may be changing sender layout concurrently; this
             * owner records a start epoch and drops that frame instead of ever
             * waiting for its control/sender mutex. */
            if (pcm_frames == 0) {
                mic_owner_apply_dsp_config();
                mic_owner_apply_spatial_commit();
                pcm_dispatch_allowed=!mic_owner_dispatch_transition_active();
                pcm_dispatch_epoch=mic_frame_dispatch_epoch_load();
            }
            if (!mic_filter_and_decimate(raw_left, raw_right, &pcm_left, &pcm_right)) {
                continue;
            }

            pcm_buf[pcm_frames * MIC_CHANNEL_COUNT] = pcm_left;
            pcm_buf[pcm_frames * MIC_CHANNEL_COUNT + 1U] = pcm_right;
            pcm_frames++;

            if (pcm_frames == MIC_OUTPUT_FRAMES_PER_PACKET) {
                dual_end_audio_analysis_metadata_t analysis = {0};
                /* Asynchronous analysis belongs to its raw-frame sequence.
                 * Never attach a previous FFT result to the current wire frame.
                 * Version zero is unavailable, not a measured healthy result. */

                /* Complete 16kHz block after existing per-channel DSP, before
                 * shared device VAD/codec. Fused PCM is dual_mono, not raw L/R.
                 * Mode 0 is an untouched branch with no estimation/NLMS work. */
                uint32_t spatial_elapsed_us=0;
                mic_nlms_block_t nlms_block = {0};
                dual_end_spatial_wiener_timing_t wiener_timing={0};
                if(s_spatial->stats.mode) {
                    int64_t spatial_started=esp_timer_get_time();
                    (void)dual_end_spatial_process_timed(s_spatial,pcm_buf,
                        MIC_OUTPUT_FRAMES_PER_PACKET,esp_timer_get_time,&wiener_timing);
                    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-spatial_started);
                    spatial_elapsed_us=elapsed;
                    portENTER_CRITICAL(&s_config_lock);
                    s_spatial_status=s_spatial->stats;
                    s_spatial_work_us+=elapsed;++s_spatial_work_frames;
                    if(elapsed>s_spatial_work_max_us)s_spatial_work_max_us=elapsed;
                    portEXIT_CRITICAL(&s_config_lock);
                    nlms_block = mic_nlms_observe(&s_spatial->stats);
                }
                mic_vad_process(pcm_buf);
                stats_output_frames += MIC_OUTPUT_FRAMES_PER_PACKET;
                for (size_t output_frame = 0;
                     output_frame < MIC_OUTPUT_FRAMES_PER_PACKET;
                     output_frame++) {
                    for (size_t channel = 0; channel < MIC_CHANNEL_COUNT; channel++) {
                        const int16_t sample =
                            pcm_buf[output_frame * MIC_CHANNEL_COUNT + channel];
                        const int32_t magnitude = abs_i16_to_i32(sample);
                        if (magnitude > pcm_peak[channel]) {
                            pcm_peak[channel] = magnitude;
                        }
                        const uint64_t sample_sq=(uint64_t)((int32_t)sample * sample);
                        pcm_sum_sq[channel] += (int64_t)sample_sq;
                        capture_health.output.sum_sq[channel]=mic_observability_u64_add(
                            capture_health.output.sum_sq[channel],sample_sq);
                        if ((uint32_t)magnitude>capture_health.output.peak[channel])
                            capture_health.output.peak[channel]=(uint32_t)magnitude;
                    }
                }

                produced_packets++;
                produced_sample_values += MIC_OUTPUT_SAMPLE_VALUES_PER_PACKET;
                mic_capture_health_complete_block(&capture_health);
                capture_health.output.samples=MIC_OUTPUT_FRAMES_PER_PACKET;
                capture_health.output.limited_frames=(uint32_t)(s_soft_limited_frames-limited_before);
                /* Consume actual async sender timing before deriving this
                 * block's spatial identity.  This task remains the sole
                 * MIC_PIPELINE runtime owner; the worker never touches DSP
                 * state or pipeline windows. */
                mic_drain_owner_timing_events();
                mic_spatial_cache_snapshot();
                dual_end_spatial_telemetry_t dispatch_identity;
                portENTER_CRITICAL(&s_config_lock);
                dispatch_identity=s_spatial_telemetry;
                portEXIT_CRITICAL(&s_config_lock);
                const int64_t callback_started_us=esp_timer_get_time();
                if (!pcm_dispatch_allowed ||
                    pcm_dispatch_epoch!=mic_frame_dispatch_epoch_load()) {
                    /* A layout or codec change was in flight at this frame
                     * boundary. Preserve the 20 ms deadline and resume with
                     * the next complete owner-local frame. */
                    portENTER_CRITICAL(&s_pipeline_lock);
                    if (s_frame_dispatch_invalidated!=UINT32_MAX) ++s_frame_dispatch_invalidated;
                    portEXIT_CRITICAL(&s_pipeline_lock);
                } else if (s_frame_callback && s_frame_dispatch_blocks) {
                    mic_frame_dispatch_t dispatch={
                        .callback=s_frame_callback,
                        .user_ctx=s_frame_user_ctx,
                        .analysis=analysis,
                        .enqueued_us=(uint64_t)callback_started_us,
                        .captured_us=processing_block->captured_us,
                        .boot_id=dispatch_identity.boot_id,
                        .capture_generation=processing_block->generation,
                        .dispatch_epoch=pcm_dispatch_epoch,
                        .spatial_generation=dispatch_identity.generation,
                        .spatial_mode=(uint8_t)dispatch_identity.applied,
                    };
                    memcpy(dispatch.pcm,pcm_buf,sizeof(dispatch.pcm));
                    if (xQueueSend(s_frame_dispatch_blocks,&dispatch,0)!=pdTRUE) {
                        portENTER_CRITICAL(&s_pipeline_lock);
                        if (s_frame_dispatch_queue_full!=UINT32_MAX) ++s_frame_dispatch_queue_full;
                        portEXIT_CRITICAL(&s_pipeline_lock);
                    } else {
                        const uint32_t depth=(uint32_t)uxQueueMessagesWaiting(s_frame_dispatch_blocks);
                        portENTER_CRITICAL(&s_pipeline_lock);
                        if (depth>s_frame_dispatch_highwater) s_frame_dispatch_highwater=depth;
                        portEXIT_CRITICAL(&s_pipeline_lock);
                    }
                } else {
                    portENTER_CRITICAL(&s_pipeline_lock);
                    if (s_frame_dispatch_unavailable!=UINT32_MAX) ++s_frame_dispatch_unavailable;
                    portEXIT_CRITICAL(&s_pipeline_lock);
                }
                const int64_t callback_finished_us=esp_timer_get_time();
                const uint32_t callback_us=callback_finished_us<=callback_started_us ? 0U :
                    ((uint64_t)(callback_finished_us-callback_started_us)>UINT32_MAX ? UINT32_MAX :
                     (uint32_t)(callback_finished_us-callback_started_us));
                frame_processing_us += callback_finished_us - processing_started_us;
                if(frame_processing_us>MIC_PROCESSING_BUDGET_US) {
                    portENTER_CRITICAL(&s_pipeline_lock);
                    ++s_pipeline_deadline_misses;s_pipeline_last_overrun_us=(uint64_t)callback_finished_us;
                    portEXIT_CRITICAL(&s_pipeline_lock);
                }
                /* d is the remaining timed owner work, not another full-frame
                 * timer.  w includes DSP, spatial processing and the bounded
                 * queue copy; asynchronous codec/radio timing is published
                 * separately when its matching completion arrives. */
                const int64_t dsp_us=frame_processing_us-spatial_elapsed_us-callback_us;
                mic_pipeline_record_block(processing_block,dequeued_us,
                    dsp_us>0 ? (uint32_t)dsp_us : 0,spatial_elapsed_us,(uint32_t)frame_processing_us,
                    &capture_health,NULL,&nlms_block,&wiener_timing);
                frame_processing_us = 0;
                processing_started_us = esp_timer_get_time();
                pcm_frames = 0;
                pcm_dispatch_allowed=false;
            }
        }
        /* Reuse the raw health accumulators instead of scanning every sample
         * twice. This includes raw blocks even when output was not dispatched. */
        for (unsigned channel=0;channel<MIC_CHANNEL_COUNT;++channel) {
            raw_sum_sq[channel]+=(int64_t)capture_health.sum_sq[channel];
            if ((int32_t)capture_health.peak[channel]>raw_peak[channel])
                raw_peak[channel]=(int32_t)capture_health.peak[channel];
        }
        frame_processing_us += esp_timer_get_time() - processing_started_us;

        TickType_t now = xTaskGetTickCount();
        if ((now - last_stats_tick) >= pdMS_TO_TICKS(1000)) {
            portENTER_CRITICAL(&s_pipeline_lock);
            analysis_frames=s_diagnostic_frames; analysis_sum_us=s_diagnostic_total_us;
            analysis_max_us=s_diagnostic_max_us;latest_analysis=s_diagnostic_metadata;
            s_diagnostic_frames=0;s_diagnostic_total_us=0;s_diagnostic_max_us=0;
            portEXIT_CRITICAL(&s_pipeline_lock);
            mic_audio_stats_t stats = {
                .input_frames = stats_input_frames,
                .output_frames = stats_output_frames,
                .bytes_read = stats_bytes_read,
                .left_raw_peak = raw_peak[0],
                .right_raw_peak = raw_peak[1],
                .left_pcm_peak = pcm_peak[0],
                .right_pcm_peak = pcm_peak[1],
                .soft_limited_frames = s_soft_limited_frames,
                .minimum_gain_permille =
                    (uint16_t)(s_soft_limit_min_gain * 1000.0f + 0.5f),
                .packets_per_sec = produced_packets - last_produced_packets,
                .sample_values_per_sec =
                    produced_sample_values - last_produced_sample_values,
                .analysis_frames = analysis_frames,
                .analysis_average_us = analysis_frames ?
                    (uint32_t)(analysis_sum_us / analysis_frames) : 0,
                .analysis_max_us = analysis_max_us,
                .latest_analysis = latest_analysis,
            };
            /* Read the applied state once with the same lock used by control
             * GETs. Configuration and gate state are at snapshot time; RMS is
             * over the preceding interval, which may include a setting change.
             * Do not report an enabled VAD as disabled or requested gain as
             * applied gain. The callback remains optional and disabled by the
             * existing runtime-log policy; no extra per-frame logging. */
            portENTER_CRITICAL(&s_config_lock);
            stats.output_gain_x = (uint8_t)s_config[2];
            stats.right_output_gain_x = (uint8_t)s_config[3];
            stats.vad_enabled = s_config[1] != 0;
            stats.vad_open = s_vad_open;
            stats.spatial = s_spatial->stats;
            stats.spatial_requested_mode = s_spatial_requested_mode;
            stats.spatial_processing_average_us = s_spatial_work_frames ? (uint32_t)(s_spatial_work_us/s_spatial_work_frames) : 0;
            stats.spatial_processing_max_us = s_spatial_work_max_us;
            portEXIT_CRITICAL(&s_config_lock);
            /* Publish sufficient statistics by value; background sqrt/logging
             * must not hold up the next 20 ms frame, even if FFT is disabled. */
            portENTER_CRITICAL(&s_pipeline_lock);
            s_stats_snapshot.stats=stats;
            memcpy(s_stats_snapshot.raw_sum_sq,raw_sum_sq,sizeof(raw_sum_sq));
            memcpy(s_stats_snapshot.pcm_sum_sq,pcm_sum_sq,sizeof(pcm_sum_sq));
            ++s_stats_snapshot.sequence;
            portEXIT_CRITICAL(&s_pipeline_lock);

            last_stats_tick = now;
            last_produced_packets = produced_packets;
            last_produced_sample_values = produced_sample_values;

            memset(raw_sum_sq, 0, sizeof(raw_sum_sq));
            memset(pcm_sum_sq, 0, sizeof(pcm_sum_sq));
            memset(raw_peak, 0, sizeof(raw_peak));
            memset(pcm_peak, 0, sizeof(pcm_peak));
            stats_input_frames = 0;
            stats_output_frames = 0;
            stats_bytes_read = 0;
            analysis_frames = 0;
            analysis_sum_us = 0;
            analysis_max_us = 0;
            s_soft_limited_frames = 0;
            s_soft_limit_min_gain = 1.0f;

        }
    }
}

esp_err_t mic_audio_init(void)
{
    /* The spatial state costs ~35 KB and is only touched by modes 1..5; keep it
     * in PSRAM so the default bypass capture leaves internal RAM free. */
    s_spatial = heap_caps_calloc(1, sizeof(*s_spatial), MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(s_spatial != NULL, ESP_ERR_NO_MEM, TAG,
                        "allocate spatial state in PSRAM");
    s_mic_state_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_mic_state_events, ESP_ERR_NO_MEM, TAG, "create state events failed");

    esp_err_t gain_ret = mic_audio_load_output_gain();
    if (gain_ret != ESP_OK) {
        ESP_LOGW(TAG,
                 "cannot load persisted fixed gain (%s); using default=%dx",
                 esp_err_to_name(gain_ret),
                 CONFIG_MIC_OUTPUT_GAIN_X);
    }
    memcpy(s_runtime_fir,s_decimator_coeff,sizeof(s_decimator_coeff));
    s_config[2]=s_config[3]=s_requested_config[2]=s_requested_config[3]=s_output_gain_x;
    s_analysis_available = mic_analysis_init() == ESP_OK;
    if (!s_analysis_available) ESP_LOGW(TAG,"FFT PSRAM unavailable; diagnostics disabled");
    mic_filter_reset();
    ESP_RETURN_ON_ERROR(mic_pdm_init(), TAG, "PDM microphone init failed");
    ESP_RETURN_ON_ERROR(mic_warmup_discard(),TAG,"PDM warmup incomplete");
    mic_filter_reset();
    xEventGroupSetBits(s_mic_state_events, MIC_STATE_ACTIVE_BIT);

    ESP_LOGI(TAG, "wiring: MCLK/PDM_CLK=GPIO%d DATA=GPIO%d; BCLK/WS/RST unused",
             MIC_PDM_CLK_GPIO, MIC_PDM_DATA_GPIO);
    return ESP_OK;
}

esp_err_t mic_audio_load_output_gain(void)
{
    s_output_gain_x = CONFIG_MIC_OUTPUT_GAIN_X;
    s_requested_output_gain_x = CONFIG_MIC_OUTPUT_GAIN_X;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MIC_CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "fixed gain=%ux (Kconfig default; no NVS override)",
                 (unsigned)s_output_gain_x);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "open microphone gain config");

    uint8_t stored_gain_x = 0;
    ret = nvs_get_u8(handle, MIC_CONFIG_NVS_GAIN_KEY, &stored_gain_x);
    nvs_close(handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "fixed gain=%ux (Kconfig default; no NVS override)",
                 (unsigned)s_output_gain_x);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "read microphone gain config");

    if (stored_gain_x < DUAL_END_MIC_GAIN_MIN_X ||
        stored_gain_x > DUAL_END_MIC_GAIN_MAX_X) {
        ESP_LOGW(TAG,
                 "ignore invalid stored fixed gain=%u; using default=%dx",
                 (unsigned)stored_gain_x,
                 CONFIG_MIC_OUTPUT_GAIN_X);
        return ESP_OK;
    }

    s_output_gain_x = stored_gain_x;
    s_requested_output_gain_x = stored_gain_x;
    ESP_LOGI(TAG, "fixed gain=%ux loaded from NVS", (unsigned)stored_gain_x);
    return ESP_OK;
}

esp_err_t mic_audio_set_output_gain(uint8_t gain_x, bool persist)
{
    ESP_RETURN_ON_FALSE(gain_x >= DUAL_END_MIC_GAIN_MIN_X &&
                            gain_x <= DUAL_END_MIC_GAIN_MAX_X,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "fixed gain must be %d..%d",
                        DUAL_END_MIC_GAIN_MIN_X,
                        DUAL_END_MIC_GAIN_MAX_X);

    if (persist) {
        nvs_handle_t handle;
        ESP_RETURN_ON_ERROR(nvs_open(MIC_CONFIG_NVS_NAMESPACE,
                                     NVS_READWRITE,
                                     &handle),
                            TAG,
                            "open microphone gain config for write");
        esp_err_t ret = nvs_set_u8(handle, MIC_CONFIG_NVS_GAIN_KEY, gain_x);
        if (ret == ESP_OK) {
            ret = nvs_commit(handle);
        }
        nvs_close(handle);
        ESP_RETURN_ON_ERROR(ret, TAG, "persist microphone gain config");
    }

    s_requested_output_gain_x = gain_x;
    portENTER_CRITICAL(&s_config_lock);
    s_requested_config[2]=s_requested_config[3]=gain_x; ++s_config_generation;
    portEXIT_CRITICAL(&s_config_lock);
    if (s_mic_task_handle == NULL) {
        s_output_gain_x = gain_x;
    } else {
        /* Wake only the DSP owner; this local gain path never needs the
         * asynchronous sender/config worker. */
        xTaskNotifyGive(s_mic_task_handle);
    }
    ESP_LOGI(TAG,
             "fixed gain set to %ux (%.2fdB)%s",
             (unsigned)gain_x,
             (double)(20.0f * log10f((float)gain_x)),
             persist ? " and saved" : "");
    return ESP_OK;
}

uint8_t mic_audio_get_output_gain(void)
{
    return s_requested_output_gain_x;
}

void mic_audio_set_frame_callback(mic_audio_frame_cb_t callback, void *user_ctx)
{
    s_frame_callback = callback;
    s_frame_user_ctx = user_ctx;
}

void mic_audio_set_stats_callback(mic_audio_stats_cb_t callback, void *user_ctx)
{
    s_stats_callback = callback;
    s_stats_user_ctx = user_ctx;
}

esp_err_t mic_audio_start(void)
{
    ESP_RETURN_ON_FALSE(s_mic_state_events && !s_mic_task_handle, ESP_ERR_INVALID_STATE,
                        TAG, "mic initialization required; task must not already run");
    xEventGroupClearBits(s_mic_state_events, MIC_START_READY_BIT | MIC_START_FAILED_BIT);
#if MIC_TASK_PSRAM_STACK
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(mic_audio_task, "mic_audio_task",
        MIC_TASK_STACK_BYTES, NULL, 5, &s_mic_task_handle, DUAL_END_AUDIO_TASK_CORE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    BaseType_t ok = xTaskCreatePinnedToCore(mic_audio_task, "mic_audio_task",
        MIC_TASK_STACK_BYTES, NULL, 5, &s_mic_task_handle, DUAL_END_AUDIO_TASK_CORE);
#endif
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create mic task failed");
    xTaskNotifyGive(s_mic_task_handle);
    EventBits_t bits = xEventGroupWaitBits(s_mic_state_events,
        MIC_START_READY_BIT | MIC_START_FAILED_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(MIC_START_TIMEOUT_MS));
    if (bits & MIC_START_FAILED_BIT) {
        TaskHandle_t failed_task = s_mic_task_handle;
        s_mic_task_handle = NULL;
#if MIC_TASK_PSRAM_STACK
        vTaskDeleteWithCaps(failed_task);
#else
        vTaskDelete(failed_task);
#endif
        return ESP_ERR_NO_MEM;
    }
    /* On timeout do not delete a task in the middle of malloc. The existing
     * mandatory startup ESP_ERROR_CHECK aborts/reboots with capture unconfirmed.
     * Keep the handle to reject accidental duplicate starts until that reset. */
    ESP_RETURN_ON_FALSE(bits & MIC_START_READY_BIT, ESP_ERR_TIMEOUT, TAG,
                        "microphone buffers did not become ready before startup deadline");
    ESP_LOGI(TAG, "capture ready: stack=%uB storage=%s core=%u",
        (unsigned)MIC_TASK_STACK_BYTES, MIC_TASK_PSRAM_STACK ? "psram" : "internal",
        (unsigned)DUAL_END_AUDIO_TASK_CORE);
    return ESP_OK;
}

esp_err_t mic_audio_set_enabled(bool enabled)
{
    ESP_RETURN_ON_FALSE(s_mic_task_handle && s_mic_state_events,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "mic task is not running");

    /* No-op must not wait for a transition the audio owner will not perform. */
    EventBits_t current=xEventGroupGetBits(s_mic_state_events);
    if(enabled && s_capture_enabled && s_capture_io_running && !(current & MIC_STATE_FAULT_BIT) && (current & MIC_STATE_ACTIVE_BIT))return ESP_OK;
    if(!enabled && !s_capture_enabled && (current & MIC_STATE_SUSPENDED_BIT))return ESP_OK;
    xEventGroupClearBits(s_mic_state_events,MIC_STATE_FAULT_BIT | MIC_STATE_ACTIVE_BIT | MIC_STATE_SUSPENDED_BIT);
    s_capture_enabled = enabled;
    xTaskNotifyGive(s_mic_task_handle);

    EventBits_t expected_bit = enabled ? MIC_STATE_ACTIVE_BIT : MIC_STATE_SUSPENDED_BIT;
    EventBits_t bits = xEventGroupWaitBits(s_mic_state_events,
                                           expected_bit | MIC_STATE_FAULT_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(MIC_STATE_TIMEOUT_MS));
    ESP_RETURN_ON_FALSE(!(bits & MIC_STATE_FAULT_BIT),ESP_FAIL,TAG,"mic hardware transition/capture failed");
    ESP_RETURN_ON_FALSE((bits & expected_bit) != 0,
                        ESP_ERR_TIMEOUT,
                        TAG,
                        "mic state change timed out");
    return s_state_change_result;
}

void mic_audio_set_motor_active(bool active)
{
    s_motor_active = active;
}

/* Runtime logger only: short counter-copy lock, then stack queries outside it. */
bool mic_audio_get_runtime_io(uint32_t values[8])
{
    if(!values || !s_capture_blocks || !s_capture_task || !s_mic_task_handle)return false;
    values[0]=(uint32_t)uxQueueMessagesWaiting(s_capture_blocks);
    portENTER_CRITICAL(&s_pipeline_lock);
    values[1]=s_pipeline_highwater;values[2]=s_dma_overflows;
    values[3]=s_capture_overloads;values[4]=s_diagnostic_drops;
    values[5]=s_pipeline_deadline_misses;
    portEXIT_CRITICAL(&s_pipeline_lock);
    values[6]=(uint32_t)uxTaskGetStackHighWaterMark(s_capture_task);
    values[7]=(uint32_t)uxTaskGetStackHighWaterMark(s_mic_task_handle);
    return true;
}

bool mic_audio_copy_diagnostics_snapshot(mic_audio_diagnostics_snapshot_t *snapshot)
{
    if (!snapshot) return false;

    /* Read the complete persistent bundle, not s_pipeline_report_ready.  The
     * short lock is a copy-only operation and is never taken by an ISR.  Do
     * not acquire the spatial/config lock here: spatial was captured at the
     * immutable pipeline close boundary, so a later snapshot cannot be joined
     * to this older cache window. */
    portENTER_CRITICAL(&s_pipeline_lock);
    const bool ready=s_pipeline_diagnostics_cache_ready;
    if (ready) *snapshot=s_pipeline_diagnostics_cache;
    portEXIT_CRITICAL(&s_pipeline_lock);
    return ready;
}
