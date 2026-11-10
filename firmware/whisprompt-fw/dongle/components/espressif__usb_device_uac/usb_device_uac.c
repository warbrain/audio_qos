/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "dual_end_runtime.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_private/usb_phy.h"
#include "esp_timer.h"
#include "tusb.h"
#include "uac_config.h"
#include "usb_device_uac.h"
#include "uac_descriptors.h"

static const char *TAG = "usbd_uac";

const uint32_t sample_rates[] = {DEFAULT_SAMPLE_RATE};

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

enum {
    VOLUME_CTRL_0_DB = 0,
    VOLUME_CTRL_10_DB = 2560,
    VOLUME_CTRL_20_DB = 5120,
    VOLUME_CTRL_30_DB = 7680,
    VOLUME_CTRL_40_DB = 10240,
    VOLUME_CTRL_50_DB = 12800,
    VOLUME_CTRL_60_DB = 15360,
    VOLUME_CTRL_70_DB = 17920,
    VOLUME_CTRL_80_DB = 20480,
    VOLUME_CTRL_90_DB = 23040,
    VOLUME_CTRL_100_DB = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};

// Resolution per format
const uint8_t spk_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
const uint8_t mic_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX};

typedef struct {
    usb_phy_handle_t phy_hdl;
    uac_device_config_t user_cfg;
    int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];         // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];      // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    uint8_t mic_buf1[MIC_INTERVAL_MS * (DEFAULT_SAMPLE_RATE / 1000) * MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX];
    uint8_t mic_buf2[MIC_INTERVAL_MS * (DEFAULT_SAMPLE_RATE / 1000) * MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX];
    uint8_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ];      // Buffer for speaker data
    uint8_t *mic_buf_write;                                      // Pointer to the buffer to write to
    uint8_t *mic_buf_read;                                       // Pointer to the buffer to read from
    int spk_data_size;                                           // Speaker data size received in the last frame
    int mic_data_size;
    size_t mic_data_offset;
    uint32_t mic_generation;
    bool mic_reset_pending;
    bool mic_resume_pending;
    bool mic_producer_busy;
    bool mic_drain_fenced;
    bool mic_expect_startup_zlp;
    uint64_t mic_written_bytes;
    uint64_t mic_completed_bytes;
    uint64_t mic_drain_fence;
    bool mic_boundary_input_active;
    bool mic_clock_ready;
    int64_t mic_bootstrap_us;
    bool mic_first_audio_pending;
    size_t mic_first_audio_offset_samples;
    int64_t mic_boundary_playout_us;
    uac_device_stats_t mic_stats;
    int spk_itf_num;
    int mic_itf_num;
    uint8_t spk_resolution;
    uint8_t mic_resolution;
    uint32_t current_sample_rate;                                // Current resolution, update on format change
    TaskHandle_t mic_task_handle;
    TaskHandle_t spk_task_handle;
    size_t spk_bytes_per_ms;
    size_t mic_bytes_per_ms;
    bool spk_active;
    bool mic_active;
} uac_device_t;

static uac_device_t *s_uac_device = NULL;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
#define UAC_ENTER_CRITICAL()    portENTER_CRITICAL_SAFE(&s_mux)
#define UAC_EXIT_CRITICAL()     portEXIT_CRITICAL_SAFE(&s_mux)

/* Caller owns s_mux. Only the USB callback touches the FIFO; producers publish
 * a separate pending block. A reset never reuses a producer's write buffer. */
static void mic_prefill_locked(void)
{
    tud_audio_clear_ep_in_ff();
    tu_fifo_t *fifo = tud_audio_get_ep_in_ff();
    uint16_t depth = fifo ? tu_fifo_depth(fifo) : 0;
    const uint16_t frame = MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX;
    uint16_t want = !s_uac_device->user_cfg.input_at_usb_boundary ?
        (uint16_t)((depth / 2 / frame) * frame) : 0;
    uint16_t filled = 0;
    static const uint8_t zeros[64] = {0};
    while (filled < want) {
        uint16_t chunk = (uint16_t)(want - filled);
        if (chunk > sizeof(zeros)) chunk = sizeof(zeros);
        uint16_t written = tud_audio_write(zeros, chunk);
        filled += written;
        if (written != chunk) break;
    }
    s_uac_device->mic_data_size = 0;
    s_uac_device->mic_data_offset = 0;
    s_uac_device->mic_written_bytes = filled;
    s_uac_device->mic_completed_bytes = 0;
    s_uac_device->mic_drain_fenced = false;
    s_uac_device->mic_drain_fence = 0;
    s_uac_device->mic_reset_pending = false;
    s_uac_device->mic_stats.mic_prefill_bytes = filled;
    s_uac_device->mic_stats.mic_fifo_depth_bytes = depth;
    s_uac_device->mic_stats.mic_fifo_level_bytes = filled;
    s_uac_device->mic_stats.mic_fifo_min_bytes = filled;
    s_uac_device->mic_first_audio_pending = false;
    s_uac_device->mic_clock_ready = false;
    s_uac_device->mic_bootstrap_us = -1;
    s_uac_device->mic_stats.mic_startup_wait_us = 0;
    s_uac_device->mic_stats.mic_first_audio_target_us = 0;
    s_uac_device->mic_stats.mic_first_audio_completion_us = 0;
}

bool uac_device_input_playout_time(int64_t *playout_us)
{
    if (!s_uac_device || !playout_us) return false;
    UAC_ENTER_CRITICAL();
    bool valid = s_uac_device->mic_boundary_input_active;
    if (valid) *playout_us = s_uac_device->mic_boundary_playout_us;
    UAC_EXIT_CRITICAL();
    return valid;
}

void uac_device_mark_first_audio_sample(size_t offset_samples, int64_t target_us)
{
    if (!s_uac_device) return;
    UAC_ENTER_CRITICAL();
    if (s_uac_device->mic_boundary_input_active && !s_uac_device->mic_stats.mic_first_audio_target_us) {
        s_uac_device->mic_first_audio_pending = true;
        s_uac_device->mic_first_audio_offset_samples = offset_samples;
        s_uac_device->mic_stats.mic_first_audio_target_us = target_us;
    }
    UAC_EXIT_CRITICAL();
}

void uac_device_reset_mic_session(void)
{
    if (!s_uac_device) return;
    UAC_ENTER_CRITICAL();
    s_uac_device->mic_generation++;
    s_uac_device->mic_data_size = 0;
    s_uac_device->mic_data_offset = 0;
    s_uac_device->mic_drain_fenced = false;
    s_uac_device->mic_reset_pending = s_uac_device->mic_active;
    UAC_EXIT_CRITICAL();
}

void uac_device_stop_mic_stream(void)
{
    if (!s_uac_device) return;
    UAC_ENTER_CRITICAL();
    s_uac_device->mic_generation++;
    s_uac_device->mic_data_size = 0;
    s_uac_device->mic_data_offset = 0;
    s_uac_device->mic_active = false;
    s_uac_device->mic_reset_pending = false;
    s_uac_device->mic_resume_pending = false;
    s_uac_device->mic_drain_fenced = false;
    tud_audio_clear_ep_in_ff();
    s_uac_device->mic_stats.mic_fifo_level_bytes = 0;
    UAC_EXIT_CRITICAL();
}

bool uac_device_mic_session_drained(void)
{
    if (!s_uac_device) return true;
    UAC_ENTER_CRITICAL();
    bool drained = !s_uac_device->mic_active;
    if (!drained && !s_uac_device->mic_producer_busy && !s_uac_device->mic_reset_pending) {
        if (!s_uac_device->mic_drain_fenced) {
            s_uac_device->mic_drain_fence = s_uac_device->mic_written_bytes +
                (uint64_t)s_uac_device->mic_data_size;
            s_uac_device->mic_drain_fenced = true;
        }
        drained = s_uac_device->mic_completed_bytes >= s_uac_device->mic_drain_fence;
    }
    UAC_EXIT_CRITICAL();
    return drained;
}

bool tud_audio_tx_pre_load_cb(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id)
{
    (void)rhport; (void)func_id;
    if (!s_uac_device) return true;
    UAC_ENTER_CRITICAL();
    if (!s_uac_device->mic_active) {
        UAC_EXIT_CRITICAL();
        return true;
    }
    const int64_t completed_us = esp_timer_get_time();
    if (n_bytes_sent > 0) s_uac_device->mic_stats.mic_last_completion_us = completed_us;
    if (s_uac_device->mic_first_audio_pending && n_bytes_sent > 0) {
        s_uac_device->mic_stats.mic_first_audio_completion_us = completed_us +
            (int64_t)s_uac_device->mic_first_audio_offset_samples * 1000000LL /
                s_uac_device->current_sample_rate;
        s_uac_device->mic_first_audio_pending = false;
    }
    s_uac_device->mic_stats.mic_tx_done_count++;
    if (n_bytes_sent == 0) {
        if (s_uac_device->mic_expect_startup_zlp)
            s_uac_device->mic_stats.mic_startup_zlp_count++;
        else s_uac_device->mic_stats.mic_zlp_count++;
    }
    s_uac_device->mic_expect_startup_zlp = false;
    if (s_uac_device->mic_reset_pending) {
        /* n_bytes_sent belongs to the retired generation. It must not advance
         * the new session's fence. Nothing from the old FIFO is loaded next. */
        mic_prefill_locked();
    } else {
        s_uac_device->mic_completed_bytes += n_bytes_sent;
    }
    bool boundary = s_uac_device->user_cfg.input_at_usb_boundary;
    uint32_t generation = s_uac_device->mic_generation;
    size_t bytes = s_uac_device->mic_bytes_per_ms;
    uint8_t *write_buf = s_uac_device->mic_buf_write;
    if (boundary && !s_uac_device->mic_clock_ready) {
        if (s_uac_device->mic_bootstrap_us < 0) s_uac_device->mic_bootstrap_us = completed_us;
        if (n_bytes_sent > 0) {
            s_uac_device->mic_clock_ready = true;
            s_uac_device->mic_stats.mic_startup_wait_us = completed_us - s_uac_device->mic_bootstrap_us;
        } else {
            /* SET_INTERFACE starts a zero-length IN transaction. Its callback
             * can precede the first real host read by tens of ms. Clock one
             * silent packet first; only a nonempty IN completion can establish
             * the boundary for fetching real PCM, never the startup ZLP. */
            memset(write_buf, 0, bytes);
            tud_audio_clear_ep_in_ff();
            uint16_t written = tud_audio_write(write_buf, (uint16_t)bytes);
            s_uac_device->mic_written_bytes += written;
            if (written != bytes) ++s_uac_device->mic_stats.mic_short_writes;
            UAC_EXIT_CRITICAL();
            return true;
        }
    }
    if (boundary) {
        /* The previous packet just completed. This block is for the following
         * USB 1 ms IN slot, with no producer queue or FIFO prefill in between. */
        s_uac_device->mic_boundary_playout_us = completed_us + 1000LL;
        s_uac_device->mic_boundary_input_active = true;
        s_uac_device->mic_producer_busy = true;
        s_uac_device->mic_stats.mic_next_input_playout_us = completed_us + 1000LL;
    }
    UAC_EXIT_CRITICAL();
    if (boundary) {
        size_t bytes_read = 0;
        esp_err_t result = s_uac_device->user_cfg.input_cb ?
            s_uac_device->user_cfg.input_cb(write_buf, bytes, &bytes_read,
                                           s_uac_device->user_cfg.cb_ctx) : ESP_ERR_INVALID_STATE;
        if (result != ESP_OK || bytes_read > bytes) { memset(write_buf, 0, bytes); bytes_read = 0; }
        else if (bytes_read < bytes) memset(write_buf + bytes_read, 0, bytes - bytes_read);
        UAC_ENTER_CRITICAL();
        s_uac_device->mic_boundary_input_active = false;
        s_uac_device->mic_producer_busy = false;
        ++s_uac_device->mic_stats.mic_boundary_callbacks;
        if (result != ESP_OK) ++s_uac_device->mic_stats.mic_input_errors;
        else if (bytes_read != bytes) ++s_uac_device->mic_stats.mic_short_reads;
        if (s_uac_device->mic_generation != generation || !s_uac_device->mic_active) {
            memset(write_buf, 0, bytes);
            s_uac_device->mic_first_audio_pending = false;
            ++s_uac_device->mic_stats.mic_stale_reads;
        }
        tud_audio_clear_ep_in_ff();
        if (s_uac_device->mic_active) {
            uint16_t written = tud_audio_write(write_buf, (uint16_t)bytes);
            s_uac_device->mic_written_bytes += written;
            if (written != bytes) ++s_uac_device->mic_stats.mic_short_writes;
        }
        UAC_EXIT_CRITICAL();
    }
    return true;
}

static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
#if CONFIG_TINYUSB_RHPORT_HS
        .otg_speed = USB_PHY_SPEED_HIGH,
#endif
    };
    usb_new_phy(&phy_conf, &s_uac_device->phy_hdl);
}

static void tusb_device_task(void *arg)
{
    while (1) {
        tud_task();
    }
}

/* TinyUSB's ESP-IDF component owns tud_mount_cb/tud_umount_cb and forwards
 * those lifecycle events through tinyusb_config_t.event_cb. Keep UAC state
 * transitions behind these helpers so the composite-device callback remains
 * the single TinyUSB callback definition. */
void uac_device_on_usb_mount(void)
{
    if (!s_uac_device) return;
    uac_device_stop_mic_stream();
    s_uac_device->spk_active = false;
    ESP_LOGI(TAG, "USB mounted");
}

void uac_device_on_usb_unmount(void)
{
    if (s_uac_device) {
        uac_device_stop_mic_stream();
        s_uac_device->spk_active = false;
    }
    ESP_LOGI(TAG, "USB unmounted");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    if (!s_uac_device) return;
    s_uac_device->spk_active = false;
    UAC_ENTER_CRITICAL();
    if (s_uac_device->mic_active) s_uac_device->mic_resume_pending = true;
    ++s_uac_device->mic_generation;
    s_uac_device->mic_data_size = 0;
    s_uac_device->mic_data_offset = 0;
    s_uac_device->mic_active = false;
    s_uac_device->mic_reset_pending = false;
    s_uac_device->mic_producer_busy = false;
    s_uac_device->mic_drain_fenced = false;
    s_uac_device->mic_boundary_input_active = false;
    s_uac_device->mic_clock_ready = false;
    s_uac_device->mic_first_audio_pending = false;
    s_uac_device->mic_bootstrap_us = -1;
    tud_audio_clear_ep_in_ff();
    s_uac_device->mic_stats.mic_fifo_level_bytes = 0;
    UAC_EXIT_CRITICAL();
    ESP_LOGI(TAG, "USB suspended; mic_resume_pending=%u",
             (unsigned)s_uac_device->mic_resume_pending);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    if (!s_uac_device) return;
    const bool mounted = tud_mounted();
    bool restarted = false;
    UAC_ENTER_CRITICAL();
    if (mounted && s_uac_device->mic_resume_pending) {
        s_uac_device->mic_resume_pending = false;
        ++s_uac_device->mic_generation;
        mic_prefill_locked();
        s_uac_device->mic_active = true;
        s_uac_device->mic_expect_startup_zlp = true;
        restarted = true;
    }
    UAC_EXIT_CRITICAL();
    if (restarted && s_uac_device->mic_task_handle)
        xTaskNotifyGive(s_uac_device->mic_task_handle);
    ESP_LOGI(TAG, "USB resumed; microphone stream %s",
             restarted ? "restored" : "awaiting host interface open");
}

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            TU_LOG1("Clock get current freq %lu\r\n", s_uac_device->current_sample_rate);
            audio_control_cur_4_t curf = { (int32_t) tu_htole32(s_uac_device->current_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef = {
                .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
            };
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur_valid = {
            .bCur = 1
        };
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        uint32_t target_sample_rate = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;
        TU_LOG1("Clock set current freq: %ld\r\n", target_sample_rate);

        if (target_sample_rate != s_uac_device->current_sample_rate) {
            // For now, we only support one sample rate
            return false;
        }

        return true;
    } else {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void)func_id;
    (void)alt_itf;
    // Set feedback method to fifo counting
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s_uac_device->current_sample_rate;

    ESP_LOGD(TAG, "Feedback method: %d, sample freq: %"PRIu32"", feedback_param->method, feedback_param->sample_freq);
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t mute1 = {
            .bCur = s_uac_device->mute[request->bChannelNumber]
        };
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
    } else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t cur_vol = {
                .bCur = tu_htole16(s_uac_device->volume[request->bChannelNumber])
            };
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));
        s_uac_device->mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;
        TU_LOG1("Set speaker channel %d Mute: %d\r\n", request->bChannelNumber, s_uac_device->mute[request->bChannelNumber]);
        if (s_uac_device->user_cfg.set_mute_cb) {
            s_uac_device->user_cfg.set_mute_cb(s_uac_device->mute[request->bChannelNumber], s_uac_device->user_cfg.cb_ctx);
        }

        return true;
    } else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));
        s_uac_device->volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
        int volume_db = s_uac_device->volume[request->bChannelNumber] / 256; // Convert to dB
        int volume = (volume_db + 50) * 2; // Map to range 0 to 100
        TU_LOG1("Set speaker channel %d volume: %d dB (%d)\r\n", request->bChannelNumber, volume_db, volume);
        if (s_uac_device->user_cfg.set_volume_cb) {
            s_uac_device->user_cfg.set_volume_cb(volume, s_uac_device->user_cfg.cb_ctx);
        }
        return true;
    } else {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_get_request(rhport, request);
    } else {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_set_request(rhport, request, buf);
    }
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt == 0) {
        TU_LOG2("Speaker interface closed");
        s_uac_device->spk_data_size = 0;
        s_uac_device->spk_active = false;
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt == 0) {
        TU_LOG2("Microphone interface closed");
        uac_device_stop_mic_stream();
    }
#endif

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt != 0) {
        s_uac_device->spk_data_size = 0;
        s_uac_device->spk_resolution = spk_resolutions_per_format[alt - 1];
        s_uac_device->spk_active = true;
        s_uac_device->spk_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * SPEAK_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX;
        xTaskNotifyGive(s_uac_device->spk_task_handle);
        TU_LOG1("Speaker interface %d-%d opened", itf, alt);
        printf("Speaker interface %d-%d opened\n", itf, alt);
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt != 0) {
        TU_VERIFY(alt <= CFG_TUD_AUDIO_FUNC_1_N_FORMATS);
        UAC_ENTER_CRITICAL();
        s_uac_device->mic_resume_pending = false;
        s_uac_device->mic_generation++;
        s_uac_device->mic_resolution = mic_resolutions_per_format[alt - 1];
        s_uac_device->mic_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX;
        mic_prefill_locked();
        s_uac_device->mic_expect_startup_zlp = true;
        s_uac_device->mic_stats.mic_stream_opens++;
        s_uac_device->mic_active = true;
        UAC_EXIT_CRITICAL();
        xTaskNotifyGive(s_uac_device->mic_task_handle);
        TU_LOG1("Microphone interface %d-%d opened", itf, alt);
        printf("Microphone interface %d-%d opened\n", itf, alt);
    }
#endif

    return true;
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_out;
    (void)cur_alt_setting;

    static bool new_play = false;
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();

    /**
     * @brief If no data is received for a certain period, it is considered as the initiation
     *        of a new audio transmission. At this point, the FIFO data is cleared, and a segment
     *        of data is buffered in the I2S.
     */
    if (now - last_time > 100 * CONFIG_UAC_SPK_NEW_PLAY_INTERVAL) {
        new_play = true;
        tud_audio_n_clear_ep_out_ff(func_id);
    }
    last_time = now;

    int bytes_remained = tud_audio_n_available(func_id);

    size_t bytes_require = s_uac_device->spk_bytes_per_ms;

    if (new_play) {
        /*!< Buffer a segment of data in the I2S and control the data size to be half of the UAC FIFO size. */
        bytes_require = SPK_INTERVAL_MS * s_uac_device->spk_bytes_per_ms / 2;
        if (bytes_remained < bytes_require) {
            return true;
        }
        new_play = false;
    }

    s_uac_device->spk_data_size = tud_audio_n_read(func_id, s_uac_device->spk_buf, bytes_require);
    xTaskNotifyGive(s_uac_device->spk_task_handle);
    return true;
}

bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)n_bytes_sent;
    (void)ep_in;
    (void)cur_alt_setting;
    tu_fifo_t *sw_in_fifo = tud_audio_n_get_ep_in_ff(func_id);
    if (!s_uac_device || !sw_in_fifo) return true;

    // load data chunk by chunk
    UAC_ENTER_CRITICAL();
    uint16_t fifo_level = tu_fifo_count(sw_in_fifo);
    s_uac_device->mic_stats.mic_fifo_level_bytes = fifo_level;
    if (fifo_level < s_uac_device->mic_stats.mic_fifo_min_bytes)
        s_uac_device->mic_stats.mic_fifo_min_bytes = fifo_level;
    if (!s_uac_device->mic_reset_pending && s_uac_device->mic_data_size > 0) {
        uint16_t requested = (uint16_t)s_uac_device->mic_data_size;
        uint16_t available = tu_fifo_remaining(sw_in_fifo);
        /* The TinyUSB FIFO permits overwrite. Limit writes explicitly so it
         * cannot silently evict old audio; continue at the exact byte offset. */
        if (requested > available) requested = available;
        const uint16_t frame = MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX;
        requested = (uint16_t)(requested / frame * frame);
        if (requested > 0) {
            uint16_t written = tud_audio_n_write(func_id,
                s_uac_device->mic_buf_read + s_uac_device->mic_data_offset, requested);
            s_uac_device->mic_data_size -= written;
            s_uac_device->mic_data_offset += written;
            s_uac_device->mic_written_bytes += written;
            if (written < requested || s_uac_device->mic_data_size > 0)
                s_uac_device->mic_stats.mic_short_writes++;
            if (s_uac_device->mic_data_size == 0) s_uac_device->mic_data_offset = 0;
            s_uac_device->mic_stats.mic_fifo_level_bytes = tu_fifo_count(sw_in_fifo);
        }
    }
    UAC_EXIT_CRITICAL();

    return true;
}

#if SPEAK_CHANNEL_NUM
static void usb_spk_task(void *pvParam)
{
    while (1) {
        if (s_uac_device->spk_active == false) {
            ulTaskNotifyTake(pdFAIL, portMAX_DELAY);
            continue;
        }
        // clear the notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_uac_device->spk_data_size == 0) {
            continue;
        }
        // playback the data from the ring buffer chunk by chunk
        if (s_uac_device->user_cfg.output_cb) {
            s_uac_device->user_cfg.output_cb((uint8_t *)s_uac_device->spk_buf, s_uac_device->spk_data_size, s_uac_device->user_cfg.cb_ctx);
        }
        s_uac_device->spk_data_size = 0;
    }
}
#endif

#if MIC_CHANNEL_NUM
static void usb_mic_task(void *pvParam)
{
    (void)pvParam;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (s_uac_device->user_cfg.input_at_usb_boundary) {
            /* Boundary mode has exactly one producer: the IN callback. */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        if (s_uac_device->mic_active == false) {
            // clear the notification
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }
        UAC_ENTER_CRITICAL();
        bool pending = s_uac_device->mic_data_size > 0;
        if (pending) s_uac_device->mic_stats.mic_pending_overruns++;
        bool produce = s_uac_device->mic_active && !pending && !s_uac_device->mic_reset_pending;
        size_t bytes_require = MIC_INTERVAL_MS * s_uac_device->mic_bytes_per_ms;
        uint32_t generation = s_uac_device->mic_generation;
        uint8_t *write_buf = s_uac_device->mic_buf_write;
        if (produce) s_uac_device->mic_producer_busy = true;
        UAC_EXIT_CRITICAL();
        if (produce) {
            size_t bytes_read = 0;
            esp_err_t ret = s_uac_device->user_cfg.input_cb ?
                s_uac_device->user_cfg.input_cb(write_buf, bytes_require, &bytes_read,
                                               s_uac_device->user_cfg.cb_ctx) : ESP_ERR_INVALID_STATE;
            if (ret != ESP_OK) {
                memset(write_buf, 0, bytes_require);
                bytes_read = bytes_require;
            } else if (bytes_read < bytes_require) {
                memset(write_buf + bytes_read, 0, bytes_require - bytes_read);
            }
            UAC_ENTER_CRITICAL();
            s_uac_device->mic_producer_busy = false;
            if (ret != ESP_OK) s_uac_device->mic_stats.mic_input_errors++;
            else if (bytes_read != bytes_require) s_uac_device->mic_stats.mic_short_reads++;
            if (s_uac_device->mic_active && !s_uac_device->mic_reset_pending &&
                s_uac_device->mic_generation == generation) {
                s_uac_device->mic_buf_write = s_uac_device->mic_buf_read;
                s_uac_device->mic_buf_read = write_buf;
                s_uac_device->mic_data_size = (int)bytes_require;
                s_uac_device->mic_data_offset = 0;
            } else s_uac_device->mic_stats.mic_stale_reads++;
            UAC_EXIT_CRITICAL();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_INTERVAL_MS));
    }
}
#endif

esp_err_t uac_device_init(uac_device_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    if (s_uac_device != NULL) {
        ESP_LOGW(TAG, "uac device already initialized");
        return ESP_OK;
    }
    s_uac_device = calloc(1, sizeof(uac_device_t));
    ESP_RETURN_ON_FALSE(s_uac_device != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for uac device");
    s_uac_device->user_cfg.output_cb = config->output_cb;
    s_uac_device->user_cfg.input_at_usb_boundary = config->input_at_usb_boundary;
    s_uac_device->user_cfg.input_cb = config->input_cb;
    s_uac_device->user_cfg.cb_ctx = config->cb_ctx;
    s_uac_device->user_cfg.set_mute_cb = config->set_mute_cb;
    s_uac_device->user_cfg.set_volume_cb = config->set_volume_cb;
    s_uac_device->current_sample_rate = DEFAULT_SAMPLE_RATE;
    s_uac_device->mic_buf_write = s_uac_device->mic_buf1;
    s_uac_device->mic_buf_read = s_uac_device->mic_buf2;

#if CONFIG_USB_DEVICE_UAC_AS_PART
    s_uac_device->spk_itf_num = config->spk_itf_num;
    s_uac_device->mic_itf_num = config->mic_itf_num;
#else
#if SPEAK_CHANNEL_NUM
    s_uac_device->spk_itf_num = ITF_NUM_AUDIO_STREAMING_SPK;
#endif
#if MIC_CHANNEL_NUM
    s_uac_device->mic_itf_num = ITF_NUM_AUDIO_STREAMING_MIC;
#endif
#endif

    BaseType_t ret_val;
    if (!config->skip_tinyusb_init) {
        usb_phy_init();
        bool usb_init = tusb_init();
        if (!usb_init) {
            ESP_LOGE(TAG, "USB Device Stack Init Fail");
            return ESP_FAIL;
        }
        ret_val = xTaskCreatePinnedToCore(tusb_device_task, "TinyUSB", 4096, NULL, CONFIG_UAC_TINYUSB_TASK_PRIORITY,
                                          NULL, CONFIG_UAC_TINYUSB_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_TINYUSB_TASK_CORE);
        ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create TinyUSB task");
    }

#if MIC_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_mic_task, "usb_mic_task", 4096, NULL, CONFIG_UAC_MIC_TASK_PRIORITY,
                                      &s_uac_device->mic_task_handle, DUAL_END_COMM_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_mic task");
#endif

#if SPEAK_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_spk_task, "usb_spk_task", 4096, NULL, CONFIG_UAC_SPK_TASK_PRIORITY,
                                      &s_uac_device->spk_task_handle, CONFIG_UAC_SPK_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_SPK_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_spk task");
#endif

    ESP_LOGI(TAG, "UAC Device Start, Version: %d.%d.%d", USB_DEVICE_UAC_VER_MAJOR, USB_DEVICE_UAC_VER_MINOR, USB_DEVICE_UAC_VER_PATCH);
    return ESP_OK;
}

void uac_device_get_stats(uac_device_stats_t *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    if (!s_uac_device) return;
    UAC_ENTER_CRITICAL();
    *stats = s_uac_device->mic_stats;
    stats->mic_active = s_uac_device->mic_active;
    stats->mic_generation = s_uac_device->mic_generation;
    stats->mic_reset_pending = s_uac_device->mic_reset_pending;
    stats->mic_producer_busy = s_uac_device->mic_producer_busy;
    stats->mic_drain_fenced = s_uac_device->mic_drain_fenced;
    stats->mic_written_bytes = s_uac_device->mic_written_bytes;
    stats->mic_completed_bytes = s_uac_device->mic_completed_bytes;
    stats->mic_drain_fence = s_uac_device->mic_drain_fence;
    UAC_EXIT_CRITICAL();
}
