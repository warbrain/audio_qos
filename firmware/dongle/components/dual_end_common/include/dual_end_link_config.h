#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#define DUAL_END_STATIC_ASSERT static_assert
#else
#define DUAL_END_STATIC_ASSERT _Static_assert
#endif

/* 信道 6 作为双方失联后的固定会合点；协商成功后允许在 1～11 信道工作。 */
#define SCEPTER_ESPNOW_CHANNEL 6
#define DUAL_END_ESPNOW_RENDEZVOUS_CHANNEL SCEPTER_ESPNOW_CHANNEL
#define DUAL_END_ESPNOW_CHANNEL_MIN 1
#define DUAL_END_ESPNOW_CHANNEL_MAX 11
#define SCEPTER_PROTOCOL_VERSION 1
#define SCEPTER_FW_VERSION "0.5.26"

#define DUAL_END_OTA_FACTORY_KEY "factory"
#define DUAL_END_OTA_APP_KEY "app"
#define DUAL_END_OTA_IMAGE_KEY_PREFIX_APP "app-"
#define DUAL_END_OTA_IMAGE_KEY_PREFIX_FACTORY "factory-"

#define DUAL_END_AUDIO_SAMPLE_RATE_HZ 16000
#define DUAL_END_AUDIO_CHANNELS 2
#define DUAL_END_AUDIO_BITS_PER_SAMPLE 16
#define DUAL_END_AUDIO_FRAME_MS 20
/* 5 秒循环可为远近距离 RSSI 对比提供可重复的音频内容。 */
#define DUAL_END_AUDIO_LOOP_FRAMES (5000 / DUAL_END_AUDIO_FRAME_MS)
#define DUAL_END_AUDIO_FRAME_SAMPLES \
    ((DUAL_END_AUDIO_SAMPLE_RATE_HZ * DUAL_END_AUDIO_FRAME_MS) / 1000)
#define DUAL_END_AUDIO_FRAME_SAMPLE_VALUES \
    (DUAL_END_AUDIO_FRAME_SAMPLES * DUAL_END_AUDIO_CHANNELS)
#define DUAL_END_AUDIO_CHANNEL_FRAME_BYTES \
    (DUAL_END_AUDIO_FRAME_SAMPLES * (DUAL_END_AUDIO_BITS_PER_SAMPLE / 8))
#define DUAL_END_AUDIO_FRAME_BYTES \
    (DUAL_END_AUDIO_FRAME_SAMPLE_VALUES * (DUAL_END_AUDIO_BITS_PER_SAMPLE / 8))

#define DUAL_END_AUDIO_G722_BITRATE_BPS 64000
#define DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES \
    ((DUAL_END_AUDIO_G722_BITRATE_BPS / 8) * DUAL_END_AUDIO_FRAME_MS / 1000)
#define DUAL_END_AUDIO_G722_FRAME_BYTES \
    (DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES * DUAL_END_AUDIO_CHANNELS)
#define DUAL_END_AUDIO_ENCODED_FRAME_BYTES DUAL_END_AUDIO_G722_FRAME_BYTES
#define DUAL_END_ESPNOW_V1_MAX_DATA_BYTES 250
#define DUAL_END_ESPNOW_V2_MAX_DATA_BYTES 1470
#define DUAL_END_ESPNOW_MAX_DATA_BYTES 400
#define DUAL_END_AUDIO_PACKET_BYTES 400

#define DUAL_END_AUDIO_ADPCM_FRAME_BYTES 164
#define DUAL_END_AUDIO_REDUNDANT_ADPCM_PAYLOAD_BYTES \
    (DUAL_END_AUDIO_ADPCM_FRAME_BYTES * 2)
#define DUAL_END_AUDIO_REDUNDANT_PCM16_PAYLOAD_BYTES \
    (DUAL_END_AUDIO_FRAME_SAMPLES * 2)
#define DUAL_END_AUDIO_OPUS_BITRATE_BPS 32000
#define DUAL_END_AUDIO_OPUS_COMPLEXITY 0
#define DUAL_END_AUDIO_OPUS_CHANNEL_FRAME_BYTES 80
#define DUAL_END_AUDIO_OPUS_FRAME_BYTES (4 + 2 * DUAL_END_AUDIO_OPUS_CHANNEL_FRAME_BYTES)
/* Codec 5: same wire layout, higher CBR bitrate. 4 + 2*155 = 314 <= 320. */
#define DUAL_END_AUDIO_OPUS62_BITRATE_BPS 62000
#define DUAL_END_AUDIO_OPUS62_CHANNEL_FRAME_BYTES 155
#define DUAL_END_AUDIO_OPUS62_FRAME_BYTES (4 + 2 * DUAL_END_AUDIO_OPUS62_CHANNEL_FRAME_BYTES)
#define DUAL_END_AUDIO_CODEC_CAP_G722 0x01
#define DUAL_END_AUDIO_CODEC_CAP_OPUS 0x02
#define DUAL_END_AUDIO_CODEC_CAP_OPUS_62 0x04
#define DUAL_END_AUDIO_CODEC_CAPABILITIES (DUAL_END_AUDIO_CODEC_CAP_G722 | \
    DUAL_END_AUDIO_CODEC_CAP_OPUS | DUAL_END_AUDIO_CODEC_CAP_OPUS_62)
#define DUAL_END_AUDIO_OPUS_PREENCODED_FRAME_BYTES 160
#define DUAL_END_AUDIO_OPUS_MAX_FRAME_BYTES 1275
#define DUAL_END_AUDIO_REDUNDANT_OPUS_PAYLOAD_BYTES \
    (DUAL_END_AUDIO_OPUS_MAX_FRAME_BYTES * 2)

#define SCEPTER_ESPNOW_TX_POWER_QDBM 80

typedef enum {
    SCEPTER_ROLE_SCEPTER = 0,
    SCEPTER_ROLE_DONGLE = 1,
} scepter_role_t;

typedef enum {
    SCEPTER_MSG_HELLO = 0x01,
    SCEPTER_MSG_CHANNEL_CONTROL = 0x02,
    SCEPTER_MSG_AUDIO_FRAME = 0x10,
    SCEPTER_MSG_CONTROL_REQUEST = 0x20,
    SCEPTER_MSG_CONTROL_RESPONSE = 0x21,
    SCEPTER_MSG_OTA_CONTROL = 0x30,
    SCEPTER_MSG_OTA_DATA = 0x31,
    SCEPTER_MSG_OTA_ACK = 0x32,
} scepter_msg_type_t;

/* Dongle 负责选信道，Scepter 只发起重扫请求并执行协商结果。 */
#define DUAL_END_CHANNEL_CONTROL_VERSION 1
#define DUAL_END_CHANNEL_FLAG_INTERFERENCE 0x01

typedef enum {
    DUAL_END_CHANNEL_CMD_SCAN_BEGIN = 0x01,
    DUAL_END_CHANNEL_CMD_SWITCH_PROPOSE = 0x02,
    DUAL_END_CHANNEL_CMD_SWITCH_ACCEPT = 0x03,
    DUAL_END_CHANNEL_CMD_SWITCH_COMMIT = 0x04,
    DUAL_END_CHANNEL_CMD_SWITCH_CONFIRM = 0x05,
    DUAL_END_CHANNEL_CMD_SCAN_END = 0x06,
    DUAL_END_CHANNEL_CMD_RESCAN_REQUEST = 0x07,
} dual_end_channel_command_t;

typedef enum {
    DUAL_END_AUDIO_CODEC_PCM16 = 0,
    DUAL_END_AUDIO_CODEC_ADPCM_IMA4 = 1,
    DUAL_END_AUDIO_CODEC_G711_MULAW = 2,
    DUAL_END_AUDIO_CODEC_OPUS = 3,
    DUAL_END_AUDIO_CODEC_G722 = 4,
    DUAL_END_AUDIO_CODEC_OPUS_62 = 5,
} dual_end_audio_codec_t;

/* Both Opus ids share one encoder API and one decoder; only the CBR frame
 * size differs. Use this instead of comparing against a single id. */
static inline int dual_end_audio_codec_is_opus(uint8_t codec)
{
    return codec == DUAL_END_AUDIO_CODEC_OPUS || codec == DUAL_END_AUDIO_CODEC_OPUS_62;
}
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_OPUS62_FRAME_BYTES <= DUAL_END_AUDIO_ENCODED_FRAME_BYTES,
                       "Opus-62 payload must fit the encoded slot");

typedef struct {
    uint8_t protocol_version;
    uint8_t source_role;
    uint8_t message_type;
    uint8_t flags;
    uint32_t sequence;
    uint16_t payload_len;
} __attribute__((packed)) dual_end_packet_header_t;

/* Dongle 是信道协调端。协商始终从当前可用信道发起；如果新信道无法确认，
 * 双方均回退到信道 6。 */
/* generation 用于拒绝旧协商包；switch_delay_ms 保证两端按同一延时切换。 */
typedef struct {
    dual_end_packet_header_t packet;
    uint8_t control_version;
    uint8_t command;
    uint8_t channel;
    uint8_t flags;
    uint32_t generation;
    uint16_t switch_delay_ms;
    int16_t channel_score;
} __attribute__((packed)) dual_end_channel_control_packet_t;

#define DUAL_END_CHANNEL_CONTROL_PAYLOAD_BYTES \
    (sizeof(dual_end_channel_control_packet_t) - sizeof(dual_end_packet_header_t))

#define DUAL_END_CONTROL_VERSION 1
#define DUAL_END_CONTROL_MAX_DATA_BYTES 13

typedef enum {
    DUAL_END_CONTROL_CMD_UI_THINKING = 0x01,
    DUAL_END_CONTROL_CMD_UI_APPROVE = 0x02,
    DUAL_END_CONTROL_CMD_UI_DONE = 0x03,
    DUAL_END_CONTROL_CMD_MOTOR_VIBRATE = 0x04,
    DUAL_END_CONTROL_CMD_MIC_FIXED_GAIN = 0x0A,
} dual_end_control_command_t;

#define DUAL_END_CONTROL_MOTOR_VIBRATE_DATA_BYTES 2
#define DUAL_END_CONTROL_MOTOR_VIBRATE_MIN_MS 1
#define DUAL_END_CONTROL_MOTOR_VIBRATE_MAX_MS 5000
#define DUAL_END_CONTROL_MIC_GAIN_DATA_BYTES 1
#define DUAL_END_MIC_GAIN_MIN_X 1
#define DUAL_END_MIC_GAIN_MAX_X 32

/* Dongle -> Scepter 的短控制包。packet.payload_len 包含 4 字节控制头和
 * data_len 字节数据；未使用的数据字节不会通过无线发送。
 * packet.sequence 是控制请求编号。 */
typedef struct {
    dual_end_packet_header_t packet;
    uint8_t control_version;
    uint8_t command;
    uint8_t data_len;
    uint8_t reserved;
    uint8_t data[DUAL_END_CONTROL_MAX_DATA_BYTES];
} __attribute__((packed)) dual_end_control_packet_t;

#define DUAL_END_CONTROL_PAYLOAD_BASE_BYTES 4
#define DUAL_END_CONTROL_PACKET_BASE_BYTES \
    (sizeof(dual_end_control_packet_t) - DUAL_END_CONTROL_MAX_DATA_BYTES)
#define DUAL_END_CONTROL_PACKET_WIRE_BYTES(data_len) \
    (DUAL_END_CONTROL_PACKET_BASE_BYTES + (data_len))

/*
 * 音频元数据放在负载之前，使 Dongle 能在解码或转发到 USB UAC 前
 * 拒绝不兼容的音频帧。
 */
typedef struct {
    uint8_t codec;
    uint8_t channels;
    uint16_t sample_rate_hz;
    uint16_t samples_per_channel;
    uint16_t pcm_bytes_after_decode;
    uint32_t payload_crc32;
    uint16_t previous_encoded_bytes;
    uint16_t current_encoded_bytes;
} __attribute__((packed)) dual_end_audio_frame_header_t;

#define DUAL_END_SENSOR_PAYLOAD_VERSION 1
#define DUAL_END_SENSOR_STATUS_FLAG_MIC_ENABLED 0x01
#define DUAL_END_SENSOR_STATUS_FLAG_IMU_VALID 0x02
#define DUAL_END_SENSOR_STATUS_FLAG_BATTERY_VALID 0x04
#define DUAL_END_SENSOR_BATTERY_FLAG_VALID 0x01
#define DUAL_END_SENSOR_BATTERY_FLAG_CHARGING 0x02
#define DUAL_END_SENSOR_KEY_1_MASK 0x01
#define DUAL_END_SENSOR_KEY_2_MASK 0x02
#define DUAL_END_SENSOR_KEY_3_MASK 0x04

#define DUAL_END_HID_INPUT_VERSION 2
#define DUAL_END_HID_FLAG_ZOOM_IN 0x01
#define DUAL_END_HID_FLAG_ZOOM_OUT 0x02
#define DUAL_END_HID_WHEEL_BUTTON_PRESSED 0x0001
#define DUAL_END_HID_WHEEL_BUTTON_STATE_VALID 0x8000

typedef struct {
    uint8_t version;
    uint8_t flags;
    uint8_t key_state;
    uint8_t mouse_buttons;
    uint8_t key_press_counter[3];
    uint8_t reserved;
    int32_t touch_dx_total;
    int32_t touch_dy_total;
    int16_t wheel_v_total;
    int16_t wheel_h_total;
    uint16_t wheel_button_counter;
    uint16_t wheel_button_state;
} __attribute__((packed)) dual_end_hid_input_payload_t;

/* 带版本号的双麦分析遥测复用原 22 字节音频填充区。所有小数评分采用
 * 无符号 Q0.15；TDOA 采用有符号 Q8.8，以 48 kHz 输入采样点为单位。
 * 这些元数据仅用于诊断，不会改变两路 G.722 音频。 */
#define DUAL_END_AUDIO_ANALYSIS_VERSION 1
#define DUAL_END_AUDIO_ANALYSIS_FLAG_VALID          0x01
#define DUAL_END_AUDIO_ANALYSIS_FLAG_CLIP_LEFT      0x02
#define DUAL_END_AUDIO_ANALYSIS_FLAG_CLIP_RIGHT     0x04
#define DUAL_END_AUDIO_ANALYSIS_FLAG_HF_GLITCH      0x08
#define DUAL_END_AUDIO_ANALYSIS_FLAG_MOTOR_ACTIVE   0x10
#define DUAL_END_AUDIO_ANALYSIS_FLAG_MOTOR_NOISE    0x20
#define DUAL_END_AUDIO_ANALYSIS_FLAG_LOW_CONFIDENCE 0x40
#define DUAL_END_AUDIO_ANALYSIS_FLAG_CALIBRATED     0x80

typedef struct {
    uint8_t version;
    uint8_t flags;
    int16_t tdoa_samples_q8_8;
    uint16_t gcc_peak_q15;
    uint16_t confidence_q15;
    uint16_t coherence_q15;
    uint16_t left_quality_q15;
    uint16_t right_quality_q15;
    uint16_t high_frequency_q15;
    uint16_t motor_q15;
    uint16_t clip_left_count;
    uint16_t clip_right_count;
} __attribute__((packed)) dual_end_audio_analysis_metadata_t;

/* 保持 ESP-NOW v2 数据包为 400 字节，并用原填充区保存流标识和采集时序。
 * 新的随机 stream_id 使接收端无需修改协议版本或破坏与上一版的 OTA
 * 兼容性，即可区分重传和发送端重启。 */
#define DUAL_END_AUDIO_PACKET_FIXED_BYTES \
    (sizeof(dual_end_packet_header_t) + sizeof(dual_end_audio_frame_header_t) + \
     DUAL_END_AUDIO_ENCODED_FRAME_BYTES + sizeof(dual_end_hid_input_payload_t))
#define DUAL_END_ESPNOW_PADDING_BYTES \
    (DUAL_END_AUDIO_PACKET_BYTES - DUAL_END_AUDIO_PACKET_FIXED_BYTES)

/*
 * 一个完整的无线音频帧负载。硬件输入状态紧随固定长度 G.722 帧，
 * 使音频与 HID 状态按同一节奏采样和传输。
 */
typedef struct {
    uint8_t audio[DUAL_END_AUDIO_ENCODED_FRAME_BYTES];
    dual_end_hid_input_payload_t hardware;
    uint32_t stream_id;
    uint32_t capture_sample_index;
    dual_end_audio_analysis_metadata_t analysis;
} __attribute__((packed)) dual_end_audio_hardware_payload_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t posture;
    uint8_t data_status;
    uint16_t stable_ms;
} __attribute__((packed)) dual_end_imu_payload_t;

typedef struct {
    int16_t dx;
    int16_t dy;
    uint8_t buttons;
    int8_t vertical;
    int8_t horizontal;
    uint8_t event_flags;
} __attribute__((packed)) dual_end_touch_payload_t;

typedef struct {
    uint16_t voltage_mv;
    uint8_t percent;
    uint8_t flags;
} __attribute__((packed)) dual_end_battery_payload_t;

typedef struct {
    uint8_t key_state;
    uint8_t key_edge;
    int8_t wheel_delta;
    uint8_t wheel_button;
    uint16_t event_counter;
    uint16_t reserved;
} __attribute__((packed)) dual_end_key_touch_payload_t;

typedef struct {
    uint8_t version;
    uint8_t flags;
    uint16_t sensor_sequence;
    uint32_t uptime_ms;
} __attribute__((packed)) dual_end_status_payload_t;

DUAL_END_STATIC_ASSERT(sizeof(dual_end_hid_input_payload_t) == 24, "HID input payload must stay 24 bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_channel_control_packet_t) == 22,
                       "channel control packet must stay 22 bytes");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_CHANNELS == 2, "wire audio must remain stereo");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_FRAME_SAMPLES == 320,
                       "each 20 ms channel must contain 320 samples");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_G722_CHANNEL_FRAME_BYTES == 160,
                       "each G.722 channel must encode to 160 bytes");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_FRAME_BYTES == 1280,
                       "each stereo PCM frame must decode to 1280 bytes");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_G722_FRAME_BYTES == 320,
                       "each stereo G.722 frame must contain 320 bytes");
DUAL_END_STATIC_ASSERT(DUAL_END_ESPNOW_PADDING_BYTES == 30,
                       "400-byte stereo packet must retain 30 metadata bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_audio_analysis_metadata_t) == 22,
                       "audio analysis metadata must stay 22 bytes");
DUAL_END_STATIC_ASSERT(DUAL_END_CONTROL_PACKET_BASE_BYTES == 14,
                       "control packet base must stay 14 bytes");
DUAL_END_STATIC_ASSERT(
    sizeof(dual_end_audio_hardware_payload_t) +
        sizeof(dual_end_packet_header_t) +
        sizeof(dual_end_audio_frame_header_t) ==
        DUAL_END_AUDIO_PACKET_BYTES,
    "combined ESP-NOW audio packet must match configured wire size");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_PACKET_BYTES <= DUAL_END_ESPNOW_MAX_DATA_BYTES,
                       "audio packet exceeds ESP-NOW payload limit");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_PACKET_BYTES > DUAL_END_ESPNOW_V1_MAX_DATA_BYTES,
                       "stereo audio packet must use ESP-NOW v2");
DUAL_END_STATIC_ASSERT(DUAL_END_AUDIO_PACKET_BYTES <= DUAL_END_ESPNOW_V2_MAX_DATA_BYTES,
                       "audio packet exceeds ESP-NOW v2 payload limit");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_imu_payload_t) == 16, "IMU payload must stay 16 bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_touch_payload_t) == 8, "touch payload must stay 8 bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_battery_payload_t) == 4, "battery payload must stay 4 bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_key_touch_payload_t) == 8, "key/touch payload must stay 8 bytes");
DUAL_END_STATIC_ASSERT(sizeof(dual_end_status_payload_t) == 8, "status payload must stay 8 bytes");

static inline const char *scepter_role_name(scepter_role_t role)
{
    switch (role) {
    case SCEPTER_ROLE_SCEPTER:
        return "scepter";
    case SCEPTER_ROLE_DONGLE:
        return "dongle";
    default:
        return "unknown";
    }
}

#ifdef __cplusplus
}
#endif
