/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*uac_output_cb_t)(uint8_t *buf, size_t len, void *cb_ctx);
typedef esp_err_t (*uac_input_cb_t)(uint8_t *buf, size_t len, size_t *bytes_read, void *cb_ctx);
typedef void (*uac_set_mute_cb_t)(uint32_t mute, void *cb_ctx);
typedef void (*uac_set_volume_cb_t)(uint32_t volume, void *cb_ctx);

/**
 * @brief USB UAC Device Config
 *
 */
typedef struct  {
    /* Fixed 16 kHz clock: request one 1 ms block at the real IN completion
     * boundary. The callback must only read already decoded PCM, never wait. */
    bool input_at_usb_boundary;
    bool skip_tinyusb_init;                      /*!< if true, the Tinyusb and usb phy will not be initialized */
    uac_output_cb_t output_cb;                   /*!< callback function for UAC data output, if NULL, output will be disabled */
    uac_input_cb_t input_cb;                     /*!< callback function for UAC data input, if NULL, input will be disabled */
    uac_set_mute_cb_t set_mute_cb;               /*!< callback function for set mute, if NULL, the set mute request will be ignored */
    uac_set_volume_cb_t set_volume_cb;           /*!< callback function for set volume, if NULL, the set volume request will be ignored */
    void *cb_ctx;                                /*!< callback context, for user specific usage */
#if CONFIG_USB_DEVICE_UAC_AS_PART
    int spk_itf_num;                             /*!< If CONFIG_USB_DEVICE_UAC_AS_PART is enabled, you need to provide the speaker interface number */
    int mic_itf_num;                             /*!< If CONFIG_USB_DEVICE_UAC_AS_PART is enabled, you need to provide the mic interface number */
#endif
} uac_device_config_t;

/**
 * @brief Initialize the USB Audio Class (UAC) device.
 *
 * @param config Pointer to the UAC device configuration structure.
 * @return
 *       - ESP_OK on success
 *       - ESP_FAIL on failure
 */
esp_err_t uac_device_init(uac_device_config_t *config);

typedef struct {
    bool mic_active;
    uint32_t mic_stream_opens, mic_tx_done_count, mic_zlp_count, mic_startup_zlp_count;
    uint32_t mic_pending_overruns, mic_input_errors, mic_short_reads, mic_short_writes;
    uint32_t mic_prefill_bytes, mic_stale_reads;
    uint16_t mic_fifo_depth_bytes, mic_fifo_level_bytes, mic_fifo_min_bytes;
    uint32_t mic_generation;
    bool mic_reset_pending, mic_producer_busy, mic_drain_fenced;
    uint64_t mic_written_bytes, mic_completed_bytes, mic_drain_fence;
    int64_t mic_last_completion_us, mic_next_input_playout_us;
    int64_t mic_startup_wait_us; /* Startup ZLP to first nonempty IN completion. */
    int64_t mic_first_audio_target_us, mic_first_audio_completion_us;
    uint32_t mic_boundary_callbacks;
} uac_device_stats_t;

/* Valid only while input_cb is running in boundary mode. */
bool uac_device_input_playout_time(int64_t *playout_us);
void uac_device_mark_first_audio_sample(size_t offset_samples, int64_t target_us);

void uac_device_get_stats(uac_device_stats_t *stats);
/* Call after clearing the upstream ring. An already scheduled IN packet is
 * completed before the USB ISR clears queued history and prefills silence. */
void uac_device_reset_mic_session(void);
void uac_device_stop_mic_stream(void);
/* Called by the registered TinyUSB event handler, after the component's
 * tud_mount_cb/tud_umount_cb have dispatched ATTACHED/DETACHED events. */
void uac_device_on_usb_mount(void);
void uac_device_on_usb_unmount(void);
/* Call only once the upstream ring has ended and drained. Captures the final
 * producer/pending/FIFO boundary and waits for actual USB IN completions. */
bool uac_device_mic_session_drained(void);

#ifdef __cplusplus
}
#endif
