#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usb_device_uac.h"
#include "esp_timer.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "check failed line %d: %s\n", __LINE__, #x); abort(); } } while (0)
#define TAG "uac_mic_test"
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ 6800
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ 748
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX 2
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS 1
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX 0
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 1
#define USE_LINEAR_BUFFER_TX 1
#define MIC_CHANNEL_NUM 2
#define SPEAK_CHANNEL_NUM 0
#define MIC_INTERVAL_MS 10
#define DEFAULT_SAMPLE_RATE 16000
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define UAC_ENTER_CRITICAL() ((void)0)
#define UAC_EXIT_CRITICAL() ((void)0)
#define TU_LOG1(...) ((void)0)
#define TU_LOG2(...) ((void)0)
#define TU_VERIFY(condition) do { if (!(condition)) return false; } while (0)
typedef void *TaskHandle_t;
typedef void *usb_phy_handle_t;
typedef struct { uint16_t wIndex, wValue; } tusb_control_request_t;
static uint16_t tu_le16toh(uint16_t n) { return n; }
static uint8_t tu_u16_low(uint16_t n) { return (uint8_t)n; }
static const uint8_t mic_resolutions_per_format[] = {16};
static jmp_buf task_exit;
static unsigned task_delays;
TickType_t uac_test_tick;
static void ulTaskNotifyTake(int clear, uint32_t wait) { (void)clear; (void)wait; longjmp(task_exit, 1); }
static void vTaskDelayUntil(TickType_t *last, TickType_t ticks) {
    (void)last; CHECK(ticks == 10); ++task_delays; longjmp(task_exit, 1);
}
static void xTaskNotifyGive(TaskHandle_t task) { (void)task; }

typedef struct { uint8_t data[6800]; uint16_t head, count, depth; } tu_fifo_t;
typedef struct { uint8_t ep_in_alt, ep_in; uint16_t packet_sz_tx[3], ep_in_sz;
                 tu_fifo_t ep_in_ff; uint8_t lin_buf_in[68]; } audiod_function_t;
static audiod_function_t audio_function;
#define fifo audio_function.ep_in_ff
static uint16_t forced_write_limit;
static tu_fifo_t *tud_audio_get_ep_in_ff(void) { return &fifo; }
static tu_fifo_t *tud_audio_n_get_ep_in_ff(uint8_t id) { CHECK(id == 0); return &fifo; }
static uint16_t tu_fifo_depth(tu_fifo_t *ff) { return ff->depth; }
static uint16_t tu_fifo_count(tu_fifo_t *ff) { return ff->count; }
static uint16_t tu_fifo_remaining(tu_fifo_t *ff) { return ff->depth - ff->count; }
static void tud_audio_clear_ep_in_ff(void) { fifo.head = fifo.count = 0; }
static uint16_t tud_audio_n_write(uint8_t id, const void *data, uint16_t count) {
    CHECK(id == 0); CHECK(count <= tu_fifo_remaining(&fifo));
    if (forced_write_limit && count > forced_write_limit) count = forced_write_limit;
    const uint8_t *src = data;
    for (unsigned i = 0; i < count; ++i) fifo.data[(fifo.head + fifo.count + i) % fifo.depth] = src[i];
    fifo.count += count; return count;
}
static uint16_t tud_audio_write(const void *data, uint16_t count) { return tud_audio_n_write(0, data, count); }
static uint16_t tu_fifo_read_n(tu_fifo_t *ff, void *dest, uint16_t count) {
    CHECK(count <= ff->count);
    for (unsigned i = 0; i < count; ++i) ((uint8_t *)dest)[i] = ff->data[(ff->head + i) % ff->depth];
    ff->head = (ff->head + count) % ff->depth; ff->count -= count; return count;
}
static uint8_t inflight[68];
static uint16_t inflight_bytes;
static uint64_t total_completed, voice_completed;
static bool usb_mounted = true;
static bool tud_mounted(void) { return usb_mounted; }
static uint8_t audiod_get_audio_fct_idx(audiod_function_t *audio) { (void)audio; return 0; }
static uint16_t tu_min16(uint16_t a, uint16_t b) { return a < b ? a : b; }
static bool usbd_edpt_xfer(uint8_t port, uint8_t ep, const void *data, uint16_t count) {
    (void)port; (void)ep; CHECK(count <= 68); memcpy(inflight, data, count); inflight_bytes = count; return true;
}
#undef fifo
#include "uac_mic_production.h"
#define fifo audio_function.ep_in_ff

static int input_mode;
static unsigned input_calls;
static bool drained_during_produce;
static tusb_control_request_t request = {3, 1};
static esp_err_t input_callback(uint8_t *buf, size_t len, size_t *read, void *ctx) {
    (void)ctx; CHECK(len == (s_uac_device->user_cfg.input_at_usb_boundary ? 64U : 640U)); ++input_calls; memset(buf, 0x55, len); *read = len;
    if (input_mode == 1) return ESP_FAIL;
    if (input_mode == 2) { *read = 32; return ESP_OK; }
    if (input_mode == 3) uac_device_reset_mic_session();
    if (input_mode == 4) {
        request.wValue = 0; CHECK(tud_audio_set_itf_close_EP_cb(0, &request));
        request.wValue = 1; CHECK(tud_audio_set_itf_cb(0, &request));
    }
    if (input_mode == 5) drained_during_produce = uac_device_mic_session_drained();
    if (input_mode == 6) memset(buf, 0, len);
    if (s_uac_device->user_cfg.input_at_usb_boundary) {
        int64_t timestamp = 0;
        CHECK(uac_device_input_playout_time(&timestamp));
        CHECK(timestamp == (int64_t)uac_test_tick * 1000LL + 1000LL);
        if (input_mode == 7) uac_device_mark_first_audio_sample(8, timestamp + 500);
    }
    return ESP_OK;
}
static void produce(void) { if (setjmp(task_exit) == 0) usb_mic_task(NULL); }
static void tick_usb(void) {
    ++uac_test_tick;
    uint16_t completed = inflight_bytes;
    total_completed += completed;
    for (unsigned i = 0; i < completed; ++i) if (inflight[i] == 0x55) ++voice_completed;
    CHECK(audiod_tx_xfer_isr(0, &audio_function, completed));
}
static void setup(void) {
    static uac_device_t device;
    memset(&device, 0, sizeof(device)); s_uac_device = &device;
    memset(&audio_function, 0, sizeof(audio_function));
    fifo.depth = 6800;
    audio_function.ep_in_alt = 1; audio_function.ep_in = 0x81; audio_function.ep_in_sz = 68;
    audio_function.packet_sz_tx[0] = 60; audio_function.packet_sz_tx[1] = 64; audio_function.packet_sz_tx[2] = 68;
    s_uac_device->current_sample_rate = 16000; s_uac_device->mic_itf_num = 3;
    s_uac_device->mic_buf_write = s_uac_device->mic_buf1;
    s_uac_device->mic_buf_read = s_uac_device->mic_buf2;
    s_uac_device->user_cfg.input_cb = input_callback;
    uac_test_tick = 0; forced_write_limit = 0; input_mode = 0; input_calls = task_delays = 0;
    inflight_bytes = 0; total_completed = voice_completed = 0;
    request.wValue = 1; CHECK(tud_audio_set_itf_cb(0, &request));
}
static void test_prefill_and_cadence(void) {
    setup(); CHECK(fifo.count == 3400); CHECK(s_uac_device->mic_stats.mic_prefill_bytes == 3400);
    for (unsigned i = 0; i < 1000; ++i) { if (i % 10 == 0) produce(); tick_usb(); }
    CHECK(task_delays == 100); CHECK(s_uac_device->mic_stats.mic_zlp_count == 0);
    CHECK(s_uac_device->mic_stats.mic_startup_zlp_count == 1);
    CHECK(s_uac_device->mic_stats.mic_fifo_min_bytes > 2500);
    CHECK(s_uac_device->mic_stats.mic_pending_overruns == 0);
}
static void test_failed_short_and_pending_reads(void) {
    setup(); input_mode = 1; produce(); CHECK(task_delays == 1);
    CHECK(s_uac_device->mic_stats.mic_input_errors == 1);
    for (unsigned i = 0; i < 640; ++i) CHECK(s_uac_device->mic_buf_read[i] == 0);
    produce(); CHECK(input_calls == 1 && task_delays == 2);
    CHECK(s_uac_device->mic_stats.mic_pending_overruns == 1);
    tick_usb(); input_mode = 2; produce();
    CHECK(s_uac_device->mic_stats.mic_short_reads == 1);
    CHECK(s_uac_device->mic_buf_read[31] == 0x55 && s_uac_device->mic_buf_read[32] == 0);
}
static void test_partial_write_keeps_order(void) {
    setup(); produce(); forced_write_limit = 128;
    for (unsigned i = 0; i < 4; ++i) { tick_usb(); CHECK(s_uac_device->mic_data_offset == (i + 1) * 128); }
    CHECK(s_uac_device->mic_data_size == 128); tick_usb();
    CHECK(s_uac_device->mic_data_size == 0 && s_uac_device->mic_data_offset == 0);
    CHECK(s_uac_device->mic_written_bytes == 4040);
    forced_write_limit = 0; input_mode = 6;
    for (unsigned i = 0; i < 100; ++i) { if (i % 10 == 0) produce(); tick_usb(); }
    CHECK(voice_completed == 640);
}
static void test_reset_drops_history_at_hardware_boundary(void) {
    setup(); produce(); tick_usb();
    CHECK(s_uac_device->mic_written_bytes == 4040);
    uac_device_reset_mic_session(); CHECK(!uac_device_mic_session_drained());
    CHECK(s_uac_device->mic_reset_pending); unsigned calls = input_calls; produce(); CHECK(input_calls == calls);
    tick_usb(); CHECK(!s_uac_device->mic_reset_pending); CHECK(s_uac_device->mic_completed_bytes == 0);
    CHECK(s_uac_device->mic_written_bytes == 3400); CHECK(s_uac_device->mic_data_size == 0);
    for (unsigned i = 0; i < inflight_bytes; ++i) CHECK(inflight[i] == 0);
    input_mode = 6; for (unsigned i = 0; i < 100; ++i) { if (i % 10 == 0) produce(); tick_usb(); }
    CHECK(voice_completed == 0);
}
static void test_inflight_producer_cannot_cross_reset_or_reopen(void) {
    setup(); input_mode = 3; produce();
    CHECK(s_uac_device->mic_data_size == 0); CHECK(s_uac_device->mic_stats.mic_stale_reads == 1);
    tick_usb(); CHECK(s_uac_device->mic_completed_bytes == 0);
    input_mode = 4; produce();
    CHECK(s_uac_device->mic_data_size == 0); CHECK(s_uac_device->mic_stats.mic_stale_reads == 2);
    CHECK(s_uac_device->mic_stats.mic_stream_opens == 2); CHECK(fifo.count == 3400);
    request.wValue = 0; CHECK(tud_audio_set_itf_close_EP_cb(0, &request));
    CHECK(uac_device_mic_session_drained()); CHECK(fifo.count == 0);
    request.wValue = 1; CHECK(tud_audio_set_itf_cb(0, &request));
}
static void test_drain_waits_for_final_pending_and_hardware_packet(void) {
    setup(); input_mode = 5; drained_during_produce = true; produce();
    CHECK(!drained_during_produce); CHECK(!s_uac_device->mic_drain_fenced);
    CHECK(!uac_device_mic_session_drained()); CHECK(s_uac_device->mic_drain_fence == 4040);
    input_mode = 6;
    unsigned ticks = 0;
    while (!uac_device_mic_session_drained()) {
        tick_usb(); ++ticks; if (ticks % 10 == 0) produce();
        if (voice_completed < 640) CHECK(!uac_device_mic_session_drained());
        CHECK(ticks < 100);
    }
    CHECK(voice_completed == 640); CHECK(fifo.count > 0); CHECK(ticks > 50);
}
static void test_fifo_guard_small_buffer(void) {
    uint16_t sizes[] = {60, 64, 68};
    CHECK(audiod_tx_packet_size(sizes, 32, 128, 68) == 32);
    CHECK(audiod_tx_packet_size(sizes, 32, 6800, 68) == 0);
}
static void test_empty_fifo_is_not_completed_hardware(void) {
    setup(); tud_audio_clear_ep_in_ff();
    uint8_t tail[64]; memset(tail, 0x55, sizeof(tail));
    CHECK(tud_audio_write(tail, sizeof(tail)) == 64);
    s_uac_device->mic_written_bytes = 64;
    audio_function.packet_sz_tx[0] = audio_function.packet_sz_tx[2] = 64;
    CHECK(!uac_device_mic_session_drained());
    tick_usb(); CHECK(fifo.count == 0 && inflight_bytes == 64);
    CHECK(!uac_device_mic_session_drained()); CHECK(voice_completed == 0);
    tick_usb(); CHECK(uac_device_mic_session_drained()); CHECK(voice_completed == 64);
}
static void test_small_fifo_has_independent_producer_storage(void) {
    setup(); request.wValue = 0; CHECK(tud_audio_set_itf_close_EP_cb(0, &request));
    fifo.depth = 272; request.wValue = 1; CHECK(tud_audio_set_itf_cb(0, &request));
    _Static_assert(sizeof(s_uac_device->mic_buf1) == 640 && sizeof(s_uac_device->mic_buf2) == 640, "UAC producer buffers must each hold 640 bytes");
    CHECK(fifo.count == 136); produce(); CHECK(s_uac_device->mic_data_size == 640);
    for (unsigned i = 0; i < 640; ++i) CHECK(s_uac_device->mic_buf_write[i] == 0);
    for (unsigned i = 0; i < 20; ++i) tick_usb();
    CHECK(s_uac_device->mic_data_size == 0);
}
static void setup_boundary(void) {
    setup(); request.wValue = 0; CHECK(tud_audio_set_itf_close_EP_cb(0, &request));
    s_uac_device->user_cfg.input_at_usb_boundary = true;
    fifo.depth = 136; request.wValue = 1; CHECK(tud_audio_set_itf_cb(0, &request));
    CHECK(fifo.count == 0 && s_uac_device->mic_stats.mic_prefill_bytes == 0);
    CHECK(audiod_tx_xfer_isr(0, &audio_function, 0)); /* Startup ZLP completion. */
    CHECK(input_calls == 0 && inflight_bytes == 64 && fifo.count == 0);
}
static void test_boundary_has_one_slot_and_real_completion_evidence(void) {
    setup_boundary(); produce(); CHECK(input_calls == 0);
    int64_t timestamp; CHECK(!uac_device_input_playout_time(&timestamp));
    input_mode = 7; tick_usb();
    CHECK(input_calls == 1 && inflight_bytes == 64 && fifo.count == 0);
    CHECK(s_uac_device->mic_stats.mic_first_audio_target_us == 2500);
    CHECK(s_uac_device->mic_stats.mic_first_audio_completion_us == 0);
    input_mode = 0; tick_usb();
    CHECK(s_uac_device->mic_stats.mic_first_audio_completion_us == 2500);
    CHECK(voice_completed == 64);
    for (unsigned i = 0; i < 1000; ++i) {
        tick_usb(); CHECK(inflight_bytes == 64 && fifo.count == 0);
    }
    CHECK(s_uac_device->mic_stats.mic_boundary_callbacks == 1002);
    CHECK(s_uac_device->mic_stats.mic_zlp_count == 0);
}
static void test_boundary_failed_short_and_reset_reads(void) {
    setup_boundary(); input_mode = 1; tick_usb();
    for (unsigned i = 0; i < 64; ++i) CHECK(inflight[i] == 0);
    input_mode = 2; tick_usb(); CHECK(inflight[31] == 0x55 && inflight[32] == 0);
    input_mode = 3; tick_usb();
    for (unsigned i = 0; i < 64; ++i) CHECK(inflight[i] == 0);
    CHECK(s_uac_device->mic_stats.mic_stale_reads == 1);
    input_mode = 6; tick_usb(); CHECK(fifo.count == 0 && inflight_bytes == 64);
    CHECK(s_uac_device->mic_stats.mic_prefill_bytes == 0);
}
static void test_bootstrap_waits_for_nonempty_in_and_keeps_actual_error(void) {
    setup_boundary(); input_mode = 7;
    CHECK(input_calls == 0 && s_uac_device->mic_stats.mic_first_audio_target_us == 0);
    uac_test_tick = 62; tick_usb(); /* First nonzero IN is delayed by 63ms. */
    CHECK(input_calls == 1 && s_uac_device->mic_stats.mic_startup_wait_us == 63000);
    CHECK(s_uac_device->mic_stats.mic_first_audio_target_us == 64500);
    CHECK(s_uac_device->mic_stats.mic_first_audio_completion_us == 0);
    uac_test_tick = 127; input_mode = 0; tick_usb(); /* Another real gap must remain visible. */
    CHECK(s_uac_device->mic_stats.mic_first_audio_completion_us == 128500);
    CHECK(s_uac_device->mic_stats.mic_first_audio_completion_us -
        s_uac_device->mic_stats.mic_first_audio_target_us == 64000);
}
static void test_suspend_resume_restores_only_an_open_mic_stream(void) {
    setup_boundary();
    const uint32_t before = s_uac_device->mic_generation;
    tud_suspend_cb(true);
    CHECK(!s_uac_device->mic_active && s_uac_device->mic_resume_pending);
    CHECK(s_uac_device->mic_generation == before + 1);
    CHECK(!s_uac_device->mic_boundary_input_active && !s_uac_device->mic_clock_ready);
    CHECK(fifo.count == 0);
    usb_mounted = true;
    tud_resume_cb();
    CHECK(s_uac_device->mic_active && !s_uac_device->mic_resume_pending);
    CHECK(s_uac_device->mic_generation == before + 2);
    CHECK(s_uac_device->mic_expect_startup_zlp && !s_uac_device->mic_clock_ready);
    CHECK(s_uac_device->mic_stats.mic_first_audio_target_us == 0);

    /* A USB detach/unmount cancels the saved intent. Resume must wait for the
     * host to reopen the UAC alternate setting; attach alone is insufficient. */
    tud_suspend_cb(true);
    CHECK(s_uac_device->mic_resume_pending);
    uac_device_on_usb_unmount();
    CHECK(!s_uac_device->mic_active && !s_uac_device->mic_resume_pending);
    usb_mounted = false;
    tud_resume_cb();
    CHECK(!s_uac_device->mic_active);
    usb_mounted = true;
    uac_device_on_usb_mount();
    CHECK(!s_uac_device->mic_active && !s_uac_device->mic_resume_pending);
    request.wValue = 1;
    CHECK(tud_audio_set_itf_cb(0, &request));
    CHECK(s_uac_device->mic_active && !s_uac_device->mic_resume_pending);
}
int main(void) {
    test_prefill_and_cadence(); test_failed_short_and_pending_reads(); test_partial_write_keeps_order();
    test_reset_drops_history_at_hardware_boundary(); test_inflight_producer_cannot_cross_reset_or_reopen();
    test_drain_waits_for_final_pending_and_hardware_packet(); test_fifo_guard_small_buffer();
    test_empty_fifo_is_not_completed_hardware(); test_small_fifo_has_independent_producer_storage();
    test_boundary_has_one_slot_and_real_completion_evidence();
    test_boundary_failed_short_and_reset_reads();
    test_bootstrap_waits_for_nonempty_in_and_keeps_actual_error();
    test_suspend_resume_restores_only_an_open_mic_stream();
    puts("UAC mic stream tests passed (13 scenarios)"); return 0;
}
