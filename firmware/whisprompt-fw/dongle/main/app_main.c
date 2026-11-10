#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dual_end_audio.h"
#include "audio_rx_telemetry.h"
#include "dual_end_runtime.h"
#include "dual_end_codec_frame.h"
#include "dual_end_spatial.h"
#include "dual_end_link_config.h"
#include "dual_end_espnow_link.h"
#include "dual_end_ota.h"
#include "dual_end_pairing.h"
#include "dongle_uac_bridge.h"
#include "dongle_usb_device.h"
#include "dongle_diagnostics.h"
#include "rgb_led.h"

static const char *TAG = "dongle";

static bool dongle_runtime_io(uint32_t values[8])
{
    dongle_uac_bridge_stats_t stats={0};dongle_uac_bridge_get_stats(&stats);
    values[0]=stats.buffer_samples;values[1]=stats.frames_in;
    values[2]=stats.sequence_lost;values[3]=stats.silence_fills;
    values[4]=stats.overflow_drop_samples;values[5]=stats.timeouts;
    values[6]=stats.session_generation;values[7]=stats.timeline_pause_count;
    return true;
}


#define APP_TOUCH_ZOOM_IN_FLAG 0x10
#define APP_TOUCH_ZOOM_OUT_FLAG 0x20
#define REMOTE_BUTTON_COUNT 4
#define BUTTON_MAPPING_STORE_MAGIC 0x5342504du
#define BUTTON_MAPPING_STORE_VER 2
#define DONGLE_HID_MOD_LEFT_GUI 0x08
#define DONGLE_HID_MOD_RIGHT_ALT 0x40
#define DONGLE_HID_KEY_ENTER 0x28
#define DONGLE_HID_KEY_ESCAPE 0x29
#define DONGLE_HID_KEY_MINUS 0x2D
#define DONGLE_HID_KEY_EQUAL 0x2E
#define DONGLE_HID_MOUSE_BUTTON_MIDDLE 0x04
#define DONGLE_RGB_BREATH_TASK_STACK_BYTES 2048
#define DONGLE_RGB_BREATH_TASK_PRIORITY 3
#define DONGLE_RGB_BREATH_STEP_MS 20
#define DONGLE_RGB_BREATH_STEP_PERCENT 2
#define DONGLE_HEALTH_LOG_INTERVAL_MS 10000
#define KEY_PRESS_RECOVERY_MAX_EVENTS 8

typedef enum {
    REMOTE_BUTTON_ID_A = 0,
    REMOTE_BUTTON_ID_B,
    REMOTE_BUTTON_ID_PTT,
    REMOTE_BUTTON_ID_C,
} remote_button_id_t;

typedef struct {
    uint8_t modifier;
    uint8_t keycode[6];
} remote_button_mapping_t;

typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    remote_button_mapping_t mapping[REMOTE_BUTTON_COUNT];
} button_mapping_store_t;

static const remote_button_mapping_t s_default_button_map[REMOTE_BUTTON_COUNT] = {
    [REMOTE_BUTTON_ID_A] = {.modifier = 0, .keycode = {DONGLE_HID_KEY_ESCAPE, 0, 0, 0, 0, 0}},
    [REMOTE_BUTTON_ID_B] = {.modifier = 0, .keycode = {DONGLE_HID_KEY_ENTER, 0, 0, 0, 0, 0}},
    [REMOTE_BUTTON_ID_PTT] = {.modifier = DONGLE_HID_MOD_RIGHT_ALT, .keycode = {0}},
    /* C is the fixed mouse wheel + middle-button composite input. */
    [REMOTE_BUTTON_ID_C] = {.modifier = 0, .keycode = {0}},
};
static remote_button_mapping_t s_button_map[REMOTE_BUTTON_COUNT];
static bool s_button_pressed[REMOTE_BUTTON_COUNT];
static uint8_t s_last_key_state;
static uint8_t s_last_key_press_counter[3];
static bool s_key_input_tracking_initialized;
static bool s_pointer_input_tracking_initialized;
static uint8_t s_last_touch_mouse_buttons;
static int32_t s_last_hid_touch_dx_total;
static int32_t s_last_hid_touch_dy_total;
static int16_t s_last_hid_wheel_v_total;
static int16_t s_last_hid_wheel_h_total;
static uint16_t s_last_hid_wheel_button_counter;
static bool s_last_hid_wheel_button_state_valid;
static bool s_last_hid_wheel_button_pressed;
static bool s_wheel_middle_pressed;
static bool s_link_connected;
static int s_link_rssi;
static portMUX_TYPE s_link_status_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_control_sequence;

typedef struct {
    bool connected;
    int rssi;
} dongle_link_status_snapshot_t;

static bool dongle_link_status_update(bool connected, int rssi)
{
    portENTER_CRITICAL(&s_link_status_lock);
    const bool was_connected = s_link_connected;
    s_link_connected = connected;
    s_link_rssi = rssi;
    portEXIT_CRITICAL(&s_link_status_lock);
    return was_connected;
}

static dongle_link_status_snapshot_t dongle_link_status_snapshot(void)
{
    dongle_link_status_snapshot_t snapshot;
    portENTER_CRITICAL(&s_link_status_lock);
    snapshot.connected = s_link_connected;
    snapshot.rssi = s_link_rssi;
    portEXIT_CRITICAL(&s_link_status_lock);
    return snapshot;
}

static const char *remote_button_name(uint8_t button_id)
{
    static const char *const names[REMOTE_BUTTON_COUNT] = {"A", "B", "P", "C"};
    return button_id < REMOTE_BUTTON_COUNT ? names[button_id] : "?";
}


static void button_mapping_set_defaults(void)
{
    memcpy(s_button_map, s_default_button_map, sizeof(s_button_map));
}

static esp_err_t button_mapping_save(void)
{
    nvs_handle_t handle;
    button_mapping_store_t store = {
        .magic = BUTTON_MAPPING_STORE_MAGIC,
        .version = BUTTON_MAPPING_STORE_VER,
    };
    memcpy(store.mapping, s_button_map, sizeof(store.mapping));

    esp_err_t ret = nvs_open("hid_cfg", NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_blob(handle, "btn_map", &store, sizeof(store));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void button_mapping_load(void)
{
    button_mapping_set_defaults();

    nvs_handle_t handle;
    esp_err_t ret = nvs_open("hid_cfg", NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "HID button mapping: defaults");
        return;
    }

    button_mapping_store_t store = {0};
    size_t len = sizeof(store);
    ret = nvs_get_blob(handle, "btn_map", &store, &len);
    nvs_close(handle);

    if (ret == ESP_OK &&
        len == sizeof(store) &&
        store.magic == BUTTON_MAPPING_STORE_MAGIC &&
        store.version == BUTTON_MAPPING_STORE_VER) {
        memcpy(s_button_map, store.mapping, sizeof(s_button_map));
        ESP_LOGI(TAG, "HID button mapping: loaded from NVS");
    } else {
        ESP_LOGI(TAG, "HID button mapping: defaults");
    }
}
static void send_mapped_button_state(uint8_t button_id, bool pressed)
{
    if (button_id >= REMOTE_BUTTON_COUNT) {
        ESP_LOGW(TAG, "unknown remote button id=%u", (unsigned)button_id);
        return;
    }

    dongle_hid_keyboard_report_t report = {0};
    s_button_pressed[button_id] = pressed;

    for (uint8_t i = 0; i < REMOTE_BUTTON_COUNT; i++) {
        if (!s_button_pressed[i]) {
            continue;
        }

        const remote_button_mapping_t *mapping = &s_button_map[i];
        report.modifier |= mapping->modifier;
        for (size_t src = 0; src < sizeof(mapping->keycode); src++) {
            uint8_t keycode = mapping->keycode[src];
            if (keycode == 0) {
                continue;
            }

            bool already_present = false;
            for (size_t dst = 0; dst < sizeof(report.keycode); dst++) {
                if (report.keycode[dst] == keycode) {
                    already_present = true;
                    break;
                }
            }
            if (already_present) {
                continue;
            }

            bool added = false;
            for (size_t dst = 0; dst < sizeof(report.keycode); dst++) {
                if (report.keycode[dst] == 0) {
                    report.keycode[dst] = keycode;
                    added = true;
                    break;
                }
            }
            if (!added) {
                ESP_LOGW(TAG, "keyboard report full, drop keycode=%u", (unsigned)keycode);
            }
        }
    }

    ESP_LOGD(TAG, "remote button %s %s", remote_button_name(button_id), pressed ? "pressed" : "released");
    ESP_ERROR_CHECK_WITHOUT_ABORT(dongle_usb_device_send_keyboard_report(&report));
}

static void release_all_mapped_buttons(void)
{
    memset(s_button_pressed, 0, sizeof(s_button_pressed));
    dongle_hid_keyboard_report_t report = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(dongle_usb_device_send_keyboard_report(&report));
}

static void reset_key_input_tracking(void)
{
    s_last_key_state = 0;
    memset(s_last_key_press_counter, 0, sizeof(s_last_key_press_counter));
    s_key_input_tracking_initialized = false;
}

static void reset_pointer_input_tracking(void)
{
    s_pointer_input_tracking_initialized = false;
}

static bool button_mapping_is_empty(uint8_t button_id)
{
    if (button_id >= REMOTE_BUTTON_COUNT) {
        return true;
    }

    const remote_button_mapping_t *mapping = &s_button_map[button_id];
    if (mapping->modifier != 0) {
        return false;
    }
    for (size_t i = 0; i < sizeof(mapping->keycode); i++) {
        if (mapping->keycode[i] != 0) {
            return false;
        }
    }
    return true;
}

static void send_wheel_middle_button_state(bool pressed)
{
    const uint8_t base_buttons = s_last_touch_mouse_buttons & 0x03;
    s_wheel_middle_pressed = pressed;
    dongle_hid_mouse_report_t report = {
        .buttons = base_buttons |
                   (pressed ? DONGLE_HID_MOUSE_BUTTON_MIDDLE : 0),
    };

    dongle_usb_device_note_mouse_packet_received();
    ESP_ERROR_CHECK_WITHOUT_ABORT(dongle_usb_device_send_mouse_report(&report));
}

static void send_wheel_button_state(bool pressed)
{
    if (button_mapping_is_empty(REMOTE_BUTTON_ID_C)) {
        ESP_LOGD(TAG, "C wheel button -> mouse middle %s",
                 pressed ? "pressed" : "released");
        send_wheel_middle_button_state(pressed);
    } else {
        ESP_LOGD(TAG, "C wheel button -> configured HID mapping %s",
                 pressed ? "pressed" : "released");
        send_mapped_button_state(REMOTE_BUTTON_ID_C, pressed);
    }
}

static void send_wheel_button_tap(void)
{
    send_wheel_button_state(true);
    send_wheel_button_state(false);
}

static void reset_wheel_button_tracking(void)
{
    if (s_last_hid_wheel_button_state_valid &&
        s_last_hid_wheel_button_pressed) {
        send_wheel_button_state(false);
    } else if (s_wheel_middle_pressed) {
        send_wheel_middle_button_state(false);
    }
    s_last_hid_wheel_button_state_valid = false;
    s_last_hid_wheel_button_pressed = false;
}

static void process_key_input(const dual_end_hid_input_payload_t *payload)
{
    static const struct {
        uint8_t mask;
        uint8_t button_id;
    } map[] = {
        {DUAL_END_SENSOR_KEY_1_MASK, REMOTE_BUTTON_ID_A},
        {DUAL_END_SENSOR_KEY_2_MASK, REMOTE_BUTTON_ID_B},
        {DUAL_END_SENSOR_KEY_3_MASK, REMOTE_BUTTON_ID_PTT},
    };

    if (!s_key_input_tracking_initialized) {
        memcpy(s_last_key_press_counter,
               payload->key_press_counter,
               sizeof(s_last_key_press_counter));
        s_last_key_state = 0;
        for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
            if (payload->key_state & map[i].mask) {
                send_mapped_button_state(map[i].button_id, true);
            }
        }
        s_last_key_state = payload->key_state;
        s_key_input_tracking_initialized = true;
        return;
    }

    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        bool reconstructed_pressed = (s_last_key_state & map[i].mask) != 0;
        bool final_pressed = (payload->key_state & map[i].mask) != 0;
        uint8_t press_events =
            (uint8_t)(payload->key_press_counter[i] -
                      s_last_key_press_counter[i]);

        if (press_events > KEY_PRESS_RECOVERY_MAX_EVENTS) {
            ESP_LOGW(TAG,
                     "key %s counter resync old=%u new=%u delta=%u",
                     remote_button_name(map[i].button_id),
                     (unsigned)s_last_key_press_counter[i],
                     (unsigned)payload->key_press_counter[i],
                     (unsigned)press_events);
            if (reconstructed_pressed != final_pressed) {
                send_mapped_button_state(map[i].button_id, final_pressed);
            }
            s_last_key_press_counter[i] = payload->key_press_counter[i];
            continue;
        }

        for (uint8_t event_index = 0;
             event_index < press_events;
             event_index++) {
            if (reconstructed_pressed) {
                send_mapped_button_state(map[i].button_id, false);
                reconstructed_pressed = false;
            }
            send_mapped_button_state(map[i].button_id, true);
            reconstructed_pressed = true;
        }

        if (reconstructed_pressed != final_pressed) {
            send_mapped_button_state(map[i].button_id, final_pressed);
        }
        s_last_key_press_counter[i] = payload->key_press_counter[i];
    }
    s_last_key_state = payload->key_state;
}

static int8_t clamp_i8_from_i32(int32_t value)
{
    if (value > INT8_MAX) {
        return INT8_MAX;
    }
    if (value < INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t)value;
}

static int16_t clamp_i16_from_i32(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static void send_zoom_key(uint8_t keycode)
{
    dongle_hid_keyboard_report_t report = {
        .modifier = DONGLE_HID_MOD_LEFT_GUI,
        .reserved = 0,
        .keycode = {keycode, 0, 0, 0, 0, 0},
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(dongle_usb_device_send_keyboard_report(&report));
    memset(&report, 0, sizeof(report));
    ESP_ERROR_CHECK_WITHOUT_ABORT(dongle_usb_device_send_keyboard_report(&report));
}

static void process_hid_input_payload(const dual_end_hid_input_payload_t *payload)
{
    if (!payload || payload->version != DUAL_END_HID_INPUT_VERSION) {
        return;
    }

    process_key_input(payload);

    if (payload->flags & DUAL_END_HID_FLAG_ZOOM_IN) {
        send_zoom_key(DONGLE_HID_KEY_EQUAL);
    }
    if (payload->flags & DUAL_END_HID_FLAG_ZOOM_OUT) {
        send_zoom_key(DONGLE_HID_KEY_MINUS);
    }

    int32_t dx = 0;
    int32_t dy = 0;
    int16_t wheel_v = 0;
    int16_t wheel_h = 0;
    uint16_t wheel_button_events = 0;
    if (s_pointer_input_tracking_initialized) {
        dx = payload->touch_dx_total - s_last_hid_touch_dx_total;
        dy = payload->touch_dy_total - s_last_hid_touch_dy_total;
        wheel_v =
            (int16_t)(payload->wheel_v_total - s_last_hid_wheel_v_total);
        wheel_h =
            (int16_t)(payload->wheel_h_total - s_last_hid_wheel_h_total);
        wheel_button_events =
            (uint16_t)(payload->wheel_button_counter -
                       s_last_hid_wheel_button_counter);
    } else {
        /* Establish a cumulative baseline after boot/reconnect; otherwise all
         * movement since Scepter boot would appear as one cursor jump. */
        s_pointer_input_tracking_initialized = true;
    }
    bool wheel_button_state_valid =
        (payload->wheel_button_state &
         DUAL_END_HID_WHEEL_BUTTON_STATE_VALID) != 0;
    bool wheel_button_pressed =
        (payload->wheel_button_state &
         DUAL_END_HID_WHEEL_BUTTON_PRESSED) != 0;

    s_last_hid_touch_dx_total = payload->touch_dx_total;
    s_last_hid_touch_dy_total = payload->touch_dy_total;
    s_last_hid_wheel_v_total = payload->wheel_v_total;
    s_last_hid_wheel_h_total = payload->wheel_h_total;
    s_last_hid_wheel_button_counter = payload->wheel_button_counter;

    if (dx != 0 || dy != 0 || wheel_v != 0 || wheel_h != 0 ||
        payload->mouse_buttons != s_last_touch_mouse_buttons) {
        dongle_hid_mouse_report_t report = {
            .buttons = (payload->mouse_buttons & 0x03) |
                       (s_wheel_middle_pressed
                            ? DONGLE_HID_MOUSE_BUTTON_MIDDLE
                            : 0),
            /* X/Y are 16-bit relative axes in the HID descriptor. Keep the
             * complete accumulated movement, especially when an ESP-NOW
             * packet was missed and the next cumulative sample catches up. */
            .x = clamp_i16_from_i32(dx),
            .y = clamp_i16_from_i32(dy),
            .vertical = clamp_i8_from_i32(wheel_v),
            .horizontal = clamp_i8_from_i32(wheel_h),
        };
        dongle_usb_device_note_mouse_packet_received();
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            dongle_usb_device_send_mouse_report(&report));
        s_last_touch_mouse_buttons = payload->mouse_buttons & 0x03;
    }

    if (wheel_button_events > 4) {
        wheel_button_events = 4;
    }

    if (wheel_button_state_valid) {
        bool press_event_consumed = false;

        if (!s_last_hid_wheel_button_state_valid) {
            s_last_hid_wheel_button_state_valid = true;
            if (wheel_button_pressed) {
                send_wheel_button_state(true);
                press_event_consumed = true;
            }
        } else if (wheel_button_pressed !=
                   s_last_hid_wheel_button_pressed) {
            send_wheel_button_state(wheel_button_pressed);
            press_event_consumed = wheel_button_pressed;
        }

        s_last_hid_wheel_button_pressed = wheel_button_pressed;
        if (press_event_consumed && wheel_button_events > 0) {
            wheel_button_events--;
        }

        /* The counter preserves a complete short click that happened between
         * two 20 ms state samples. Do not synthesize taps while the latest
         * sampled state is held, because that would release the active hold. */
        if (!wheel_button_pressed) {
            for (uint16_t i = 0; i < wheel_button_events; i++) {
                send_wheel_button_tap();
            }
        }
    } else {
        if (s_last_hid_wheel_button_state_valid &&
            s_last_hid_wheel_button_pressed) {
            send_wheel_button_state(false);
        }
        s_last_hid_wheel_button_state_valid = false;
        s_last_hid_wheel_button_pressed = false;
        for (uint16_t i = 0; i < wheel_button_events; i++) {
            /* Compatibility with firmware that only sends the counter. */
            send_wheel_button_tap();
        }
    }
}


static esp_err_t dongle_forward_scepter_control(uint8_t command,
                                                const uint8_t *data,
                                                uint8_t data_len)
{
    if (data_len > DUAL_END_CONTROL_MAX_DATA_BYTES ||
        (data_len > 0 && data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (command) {
    case DUAL_END_CONTROL_CMD_UI_THINKING:
    case DUAL_END_CONTROL_CMD_UI_APPROVE:
    case DUAL_END_CONTROL_CMD_UI_DONE:
        if (data_len != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        break;

    case DUAL_END_CONTROL_CMD_MOTOR_VIBRATE: {
        if (data_len != DUAL_END_CONTROL_MOTOR_VIBRATE_DATA_BYTES) {
            return ESP_ERR_INVALID_SIZE;
        }
        uint16_t duration_ms = (uint16_t)data[0] |
                               ((uint16_t)data[1] << 8);
        if (duration_ms < DUAL_END_CONTROL_MOTOR_VIBRATE_MIN_MS ||
            duration_ms > DUAL_END_CONTROL_MOTOR_VIBRATE_MAX_MS) {
            return ESP_ERR_INVALID_ARG;
        }
        break;
    }

    case DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN:
        if (data_len != DUAL_END_CONTROL_MIC_GAIN_DATA_BYTES) {
            return ESP_ERR_INVALID_SIZE;
        }
        if (data[0] < DUAL_END_MIC_GAIN_MIN_X ||
            data[0] > DUAL_END_MIC_GAIN_MAX_X) {
            return ESP_ERR_INVALID_ARG;
        }
        break;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!dual_end_espnow_link_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    dual_end_control_packet_t packet = {
        .packet = {
            .protocol_version = SCEPTER_PROTOCOL_VERSION,
            .source_role = SCEPTER_ROLE_DONGLE,
            .message_type = SCEPTER_MSG_CONTROL_REQUEST,
            .sequence = s_control_sequence++,
            .payload_len = DUAL_END_CONTROL_PAYLOAD_BASE_BYTES + data_len,
        },
        .control_version = DUAL_END_CONTROL_VERSION,
        .command = command,
        .data_len = data_len,
    };
    if (data_len > 0) {
        memcpy(packet.data, data, data_len);
    }

    int wire_len = (int)DUAL_END_CONTROL_PACKET_WIRE_BYTES(data_len);
    esp_err_t ret = dual_end_espnow_link_send_control(&packet, wire_len);
    ESP_LOGI(TAG,
             "Scepter control forward cmd=0x%02x seq=%" PRIu32 " bytes=%d ret=%s",
             command,
             packet.packet.sequence,
             wire_len,
             esp_err_to_name(ret));
    return ret;
}

/* 0x11 is a backward-compatible DSP extension; old firmware rejects it.
 * Reports are bounded, never consume keyboard Input reports. */
static portMUX_TYPE s_dsp_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_dsp_reply[16];
static void dongle_dsp_response(const uint8_t *src,const uint8_t *data,int len,int rssi,void *ctx) {
    (void)rssi; (void)ctx;
    uint8_t peer[6];
    if (!src || dual_end_espnow_link_get_peer(peer,NULL)!=ESP_OK || memcmp(src,peer,6) ||
        !data || len!=DUAL_END_CONTROL_PACKET_WIRE_BYTES(13)) return;
    const dual_end_control_packet_t *p=(const dual_end_control_packet_t *)data;
    if(p->packet.protocol_version!=SCEPTER_PROTOCOL_VERSION || p->packet.source_role!=SCEPTER_ROLE_SCEPTER ||
       p->packet.message_type!=SCEPTER_MSG_CONTROL_RESPONSE || p->packet.payload_len!=DUAL_END_CONTROL_PAYLOAD_BASE_BYTES+13 ||
       p->control_version!=1 || p->reserved || p->command!=0x0B || p->data_len!=13 || !dual_end_dsp_version_item_valid(p->data[0],p->data[2]) ||
       p->data[9]>3 || p->data[10] || p->data[11] || p->data[12]) return;
    portENTER_CRITICAL(&s_dsp_lock);
    if(s_dsp_reply[0]==0x11 && s_dsp_reply[10]==1 && !memcmp(s_dsp_reply+1,p->data,7)) {
        memcpy(s_dsp_reply+1,p->data,13);
        if (p->data[0] == 2 && p->data[2] == 16 && p->data[9] == 0) {
            const uint16_t capabilities = dual_end_codec_u16(p->data + 7) &
                dual_end_audio_receiver_codec_capabilities();
            s_dsp_reply[8] = (uint8_t)capabilities;
            s_dsp_reply[9] = (uint8_t)(capabilities >> 8);
        }
        /* New processing capability is separate from codec item 16. The
         * upgraded Dongle understands dual_mono while old versions reject v3. */
        if (p->data[0] == 3 && p->data[2] == 17 && p->data[9] == 0) {
            const uint16_t support = dual_end_codec_u16(p->data + 7) & DUAL_END_SPATIAL_CAPABILITIES;
            s_dsp_reply[8] = (uint8_t)support;
            s_dsp_reply[9] = (uint8_t)(support >> 8);
        }
        dongle_usb_device_set_config_reply(s_dsp_reply);
    }
    portEXIT_CRITICAL(&s_dsp_lock);
}
static esp_err_t dongle_dsp_request(const uint8_t *data) {
    if(!dual_end_dsp_version_item_valid(data[1],data[3]) || (data[2]!=1 && data[2]!=2) || data[10] || data[11] || data[12] || data[13] || data[14] || data[15]) return ESP_ERR_INVALID_ARG;
    if (data[1] == 2 && data[2] == 2) {
        const uint16_t value = dual_end_codec_u16(data + 8);
        const uint16_t required = value == DUAL_END_AUDIO_CODEC_G722 ? DUAL_END_AUDIO_CODEC_CAP_G722 :
            value == DUAL_END_AUDIO_CODEC_OPUS ? DUAL_END_AUDIO_CODEC_CAP_OPUS :
            value == DUAL_END_AUDIO_CODEC_OPUS_62 ? DUAL_END_AUDIO_CODEC_CAP_OPUS_62 : 0;
        if (data[3] != 15 || !required ||
            !(required & dual_end_audio_receiver_codec_capabilities())) return ESP_ERR_INVALID_ARG;
    }
    if (data[1] == 3 && data[2] == 2 && (data[3] != 18 ||
        !dual_end_spatial_mode_supported(dual_end_codec_u16(data + 8),DUAL_END_SPATIAL_CAPABILITIES)))
        return ESP_ERR_INVALID_ARG;
    dual_end_control_packet_t p={.packet={.protocol_version=SCEPTER_PROTOCOL_VERSION,.source_role=SCEPTER_ROLE_DONGLE,
        .message_type=SCEPTER_MSG_CONTROL_REQUEST,.sequence=s_control_sequence++,.payload_len=DUAL_END_CONTROL_PAYLOAD_BASE_BYTES+13},
        .control_version=1,.command=0x0B,.data_len=13};
    memcpy(p.data,data+1,13);
    portENTER_CRITICAL(&s_dsp_lock);
    memcpy(s_dsp_reply,data,16);s_dsp_reply[10]=1;
    dongle_usb_device_set_config_reply(s_dsp_reply);
    portEXIT_CRITICAL(&s_dsp_lock);
    const bool link_connected = dual_end_espnow_link_is_connected();
    esp_err_t ret = link_connected ?
        dual_end_espnow_link_queue_control(&p, DUAL_END_CONTROL_PACKET_WIRE_BYTES(13)) :
        ESP_ERR_INVALID_STATE;
    if(ret!=ESP_OK) {
        const char *reason = !link_connected ? "link_disconnected" :
            ret == ESP_ERR_NO_MEM ? "control_queue_full" :
            ret == ESP_ERR_INVALID_STATE ? "radio_not_ready" : "forward_rejected";
        ESP_LOGW(TAG,
                 "DSP_FORWARD_REJECT token=%02x%02x%02x%02x item=%u op=%u"
                 " reason=%s link=%u err=%s hid_status=4",
                 (unsigned)data[7], (unsigned)data[6],
                 (unsigned)data[5], (unsigned)data[4], (unsigned)data[3],
                 (unsigned)data[2], reason, link_connected ? 1U : 0U,
                 esp_err_to_name(ret));
        portENTER_CRITICAL(&s_dsp_lock);
        s_dsp_reply[10]=4;dongle_usb_device_set_config_reply(s_dsp_reply);
        portEXIT_CRITICAL(&s_dsp_lock);
    }
    return ret;
}

static esp_err_t dongle_hid_config_report_cb(uint8_t report_id,
                                             uint8_t report_type,
                                             const uint8_t *buffer,
                                             uint16_t bufsize,
                                             void *ctx)
{
    (void)ctx;

    if (report_id != DONGLE_HID_REPORT_ID_CONFIG ||
        (report_type != DONGLE_HID_REPORT_TYPE_OUTPUT &&
         report_type != DONGLE_HID_REPORT_TYPE_FEATURE)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!buffer || bufsize < DONGLE_HID_CONFIG_REPORT_SIZE) {
        ESP_LOGW(TAG, "HID config report too short: %u", (unsigned)bufsize);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t cmd = buffer[0];
    switch (cmd) {
    case 0x11: return dongle_dsp_request(buffer);
    case DONGLE_HID_CONFIG_CMD_SET_BUTTON: {
        uint8_t button_id = buffer[1];
        if (button_id >= REMOTE_BUTTON_COUNT) {
            ESP_LOGW(TAG, "HID config invalid button id=%u", (unsigned)button_id);
            return ESP_ERR_INVALID_ARG;
        }
        remote_button_mapping_t mapping = {
            .modifier = buffer[2],
            .keycode = {
                buffer[3],
                buffer[4],
                buffer[5],
                buffer[6],
                buffer[7],
                buffer[8],
            },
        };

        reset_wheel_button_tracking();
        release_all_mapped_buttons();
        s_button_map[button_id] = mapping;
        esp_err_t ret = button_mapping_save();
        ESP_LOGI(TAG,
                 "HID config: button %s modifier=0x%02x keys=%u,%u,%u,%u,%u,%u save=%s",
                 remote_button_name(button_id),
                 mapping.modifier,
                 mapping.keycode[0],
                 mapping.keycode[1],
                 mapping.keycode[2],
                 mapping.keycode[3],
                 mapping.keycode[4],
                 mapping.keycode[5],
                 esp_err_to_name(ret));
        return ret;
    }

    case DONGLE_HID_CONFIG_CMD_RESET: {
        reset_wheel_button_tracking();
        release_all_mapped_buttons();
        button_mapping_set_defaults();
        esp_err_t ret = button_mapping_save();
        ESP_LOGI(TAG, "HID config: reset defaults save=%s", esp_err_to_name(ret));
        return ret;
    }

    case DONGLE_HID_CONFIG_CMD_FORWARD_TO_SCEPTER: {
        uint8_t command = buffer[1];
        uint8_t data_len = buffer[2];
        if (data_len > DUAL_END_CONTROL_MAX_DATA_BYTES ||
            (uint16_t)(3 + data_len) > bufsize) {
            return ESP_ERR_INVALID_SIZE;
        }
        return dongle_forward_scepter_control(command, &buffer[3], data_len);
    }

    default:
        ESP_LOGW(TAG, "HID config unknown cmd=%u", (unsigned)cmd);
        return ESP_ERR_NOT_SUPPORTED;
    }
}
static void dongle_input_event_sink(const dual_end_hid_input_payload_t *payload,
                                    uint32_t sequence,
                                    int rssi,
                                    void *ctx)
{
    (void)sequence;
    (void)rssi;
    (void)ctx;
    process_hid_input_payload(payload);
}


static void dongle_audio_session_reset_cb(void *ctx)
{
    (void)ctx;
    dongle_uac_bridge_reset_session();
    dongle_usb_device_reset_audio_session();
}

static void dongle_link_status_cb(bool connected, int rssi, void *ctx)
{
    (void)ctx;
    /* Record every transition.  The old code stored only the disconnect edge,
     * so s_link_connected stayed false forever: health reported link=0 while
     * audio flowed, and the disconnect reset below never ran because
     * was_connected could never be true. */
    const bool was_connected = dongle_link_status_update(connected, rssi);
    if (!connected && was_connected) {
        dongle_audio_session_reset_cb(NULL);
        reset_wheel_button_tracking();
        release_all_mapped_buttons();
        reset_key_input_tracking();
        reset_pointer_input_tracking();
    }
}
static void dongle_ota_activity_cb(bool active, void *ctx)
{
    (void)ctx;
    dual_end_espnow_link_set_ota_active(active);
    if (active) {
        dongle_audio_session_reset_cb(NULL);
    }
}
static void rgb_breath_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_tick = xTaskGetTickCount();
    int brightness_percent = 0;
    int breath_step = DONGLE_RGB_BREATH_STEP_PERCENT;

    ESP_LOGI(TAG, "RGB breath task started: orange period=%dms",
             (200 / DONGLE_RGB_BREATH_STEP_PERCENT) * DONGLE_RGB_BREATH_STEP_MS);
    while (true) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            rgb_led_set_orange_brightness((uint8_t)brightness_percent));

        brightness_percent += breath_step;
        if (brightness_percent >= 100) {
            brightness_percent = 100;
            breath_step = -DONGLE_RGB_BREATH_STEP_PERCENT;
        } else if (brightness_percent <= 0) {
            brightness_percent = 0;
            breath_step = DONGLE_RGB_BREATH_STEP_PERCENT;
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DONGLE_RGB_BREATH_STEP_MS));
    }
}
void app_main(void)
{
    const scepter_role_t role = SCEPTER_ROLE_DONGLE;
    ESP_LOGI(TAG, "CytoScepter dongle boot, fw=%s, rx=espnow, codecs=g722/opus, default=g722-stereo, usb=uac", SCEPTER_FW_VERSION);

    ESP_ERROR_CHECK(rgb_led_init());
    BaseType_t rgb_task_ok = xTaskCreate(rgb_breath_task,
                                         "rgb_breath",
                                         DONGLE_RGB_BREATH_TASK_STACK_BYTES,
                                         NULL,
                                         DONGLE_RGB_BREATH_TASK_PRIORITY,
                                         NULL);
    ESP_ERROR_CHECK(rgb_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(dual_end_ota_service_init(role));
    ESP_ERROR_CHECK(dual_end_ota_register_activity_cb(dongle_ota_activity_cb, NULL));
    ESP_ERROR_CHECK(dual_end_espnow_link_init(role));
    ESP_ERROR_CHECK(dual_end_espnow_link_register_status_cb(dongle_link_status_cb, NULL));
    ESP_ERROR_CHECK(dual_end_pairing_service_init(role, false));
    button_mapping_load();
    ESP_ERROR_CHECK(dongle_uac_bridge_start());
    dongle_usb_device_set_hid_config_report_cb(dongle_hid_config_report_cb, NULL);
    dongle_usb_device_set_diagnostics_command_cb(dongle_diagnostics_handle_command, NULL);
    ESP_ERROR_CHECK(dongle_usb_device_start());
    ESP_ERROR_CHECK(dual_end_espnow_link_register_control_callbacks(dongle_dsp_response,NULL,NULL));
    ESP_ERROR_CHECK(dual_end_audio_receiver_set_input_event_sink(dongle_input_event_sink, NULL));
    dual_end_audio_receiver_set_session_reset_cb(dongle_audio_session_reset_cb, NULL);
    /* Production Dongle playout is absolute-timeline only. This keeps the
     * legacy untimed PCM ring out of the target component entirely. */
    ESP_ERROR_CHECK(dual_end_audio_receiver_start_timed(
        dongle_uac_bridge_write_timed_pcm,
        dongle_uac_bridge_playout_clock,
        NULL));
    ESP_ERROR_CHECK(dual_end_ota_confirm_running_image());
    (void)dual_end_runtime_set_io_probe(dongle_runtime_io);
    if (dual_end_runtime_start(role) != ESP_OK)
        ESP_LOGW(TAG, "profiling_disabled; starting original audio/control services");
    if (dongle_diagnostics_start() != ESP_OK)
        ESP_LOGW(TAG, "diagnostics_cache_disabled; audio/control services unchanged");


    dongle_uac_bridge_stats_t previous_audio = {0};
    dongle_usb_device_stats_t previous_usb = {0};
    dongle_uac_bridge_get_stats(&previous_audio);
    dongle_usb_device_get_stats(&previous_usb);
    int64_t previous_health_us = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(DONGLE_HEALTH_LOG_INTERVAL_MS));

        dongle_uac_bridge_stats_t audio = {0};
        dongle_usb_device_stats_t usb = {0};
        dongle_uac_bridge_get_stats(&audio);
        dongle_usb_device_get_stats(&usb);
        const dongle_link_status_snapshot_t link_status = dongle_link_status_snapshot();

        int64_t now_us = esp_timer_get_time();
        uint64_t elapsed_us = (uint64_t)(now_us - previous_health_us);
        uint32_t frames = audio.frames_in - previous_audio.frames_in;
        uint32_t lost = audio.sequence_lost - previous_audio.sequence_lost;
        uint32_t expected = frames + lost;
        uint32_t rx_fps = elapsed_us > 0 ?
                          (uint32_t)(((uint64_t)frames * 1000000ULL) / elapsed_us) : 0;
        uint32_t loss_ppm = expected > 0 ?
                            (uint32_t)(((uint64_t)lost * 1000000ULL) / expected) : 0;
        uint32_t buffer_ms = (uint32_t)(((uint64_t)audio.buffer_samples * 1000ULL) /
                                        (DUAL_END_AUDIO_SAMPLE_RATE_HZ *
                                         DUAL_END_AUDIO_CHANNELS));

        static uint32_t previous_concealed_frames;
        uint32_t concealed_delta = 0;
        audio_rx_telemetry_cache_t telemetry_cache;
        if (audio_rx_telemetry_get_latest(&telemetry_cache)) {
            const uint32_t concealed = telemetry_cache.snapshot.concealed_repeat_frames;
            concealed_delta = concealed - previous_concealed_frames;
            previous_concealed_frames = concealed;
        }
        ESP_LOGI(TAG,
                 "health up=%" PRIu64 "s link=%d rssi=%d rx=%" PRIu32 "fps"
                 " loss=%" PRIu32 "/%" PRIu32 "(%" PRIu32 "ppm)"
                 " ooo=%" PRIu32 " audio=%d buf=%" PRIu32 "ms"
                 " timeout=%" PRIu32 " silence=%" PRIu32
                 " adjust=-%" PRIu32 "/+%" PRIu32 " overflow=%" PRIu32
                 " usb=%d cdc=%d hid_wait=%" PRIu32
                 " heap=%" PRIu32 " min_heap=%" PRIu32 " concealed=%" PRIu32,
                 (uint64_t)(now_us / 1000000ULL),
                 link_status.connected ? 1 : 0,
                 link_status.rssi,
                 rx_fps,
                 lost,
                 expected,
                 loss_ppm,
                 audio.sequence_reordered - previous_audio.sequence_reordered,
                 audio.rx_active ? 1 : 0,
                 buffer_ms,
                 audio.timeouts - previous_audio.timeouts,
                 audio.silence_fills - previous_audio.silence_fills,
                 audio.level_drop_samples - previous_audio.level_drop_samples,
                 audio.level_dup_samples - previous_audio.level_dup_samples,
                 audio.overflow_drop_samples - previous_audio.overflow_drop_samples,
                 usb.mounted ? 1 : 0,
                 usb.cdc_connected ? 1 : 0,
                 usb.mouse_hid_not_ready - previous_usb.mouse_hid_not_ready,
                 esp_get_free_heap_size(),
                 esp_get_minimum_free_heap_size(),
                 concealed_delta);

        previous_audio = audio;
        previous_usb = usb;
        previous_health_us = now_us;
    }
}
