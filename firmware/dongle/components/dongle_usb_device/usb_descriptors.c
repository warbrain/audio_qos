#include <stdint.h>

#include "class/hid/hid.h"
#include "dongle_usb_device.h"
#include "tusb.h"
#include "uac_descriptors.h"

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_AUDIO_CONTROL,
    ITF_NUM_AUDIO_STREAMING_MIC,
    ITF_NUM_TOTAL,
};

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
    STRID_HID,
    STRID_UAC,
    STRID_UAC_MIC,
};

enum {
    EPNUM_CDC_NOTIF = 0x81,
    EPNUM_CDC_OUT = 0x02,
    EPNUM_CDC_IN = 0x82,
    EPNUM_HID_IN = 0x83,
    EPNUM_AUDIO_IN = 0x84,
};

#define REPORT_ID_KEYBOARD DONGLE_HID_REPORT_ID_KEYBOARD
#define REPORT_ID_MOUSE DONGLE_HID_REPORT_ID_MOUSE
#define REPORT_ID_CONFIG DONGLE_HID_REPORT_ID_CONFIG
#define HID_CONFIG_REPORT_SIZE DONGLE_HID_CONFIG_REPORT_SIZE
const tusb_desc_device_t dongle_usb_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x8001,
    .bcdDevice = 0x0101,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 1,
};

const uint8_t dongle_hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    /* Hi-res relative mouse: 16-bit X/Y + 8-bit vertical wheel/horizontal pan. */
    HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP                  ),
    HID_USAGE        ( HID_USAGE_DESKTOP_MOUSE                 ),
    HID_COLLECTION   ( HID_COLLECTION_APPLICATION              ),
        HID_REPORT_ID    ( REPORT_ID_MOUSE                     )
        HID_USAGE        ( HID_USAGE_DESKTOP_POINTER           ),
        HID_COLLECTION   ( HID_COLLECTION_PHYSICAL             ),
            HID_USAGE_PAGE   ( HID_USAGE_PAGE_BUTTON           ),
                HID_USAGE_MIN    ( 1                           ),
                HID_USAGE_MAX    ( 5                           ),
                HID_LOGICAL_MIN  ( 0                           ),
                HID_LOGICAL_MAX  ( 1                           ),
                HID_REPORT_COUNT ( 5                           ),
                HID_REPORT_SIZE  ( 1                           ),
                HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
                HID_REPORT_COUNT ( 1                           ),
                HID_REPORT_SIZE  ( 3                           ),
                HID_INPUT        ( HID_CONSTANT                ),
            HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP          ),
                HID_USAGE         ( HID_USAGE_DESKTOP_X        ),
                HID_USAGE         ( HID_USAGE_DESKTOP_Y        ),
                HID_LOGICAL_MIN_N ( -32767, 2                  ),
                HID_LOGICAL_MAX_N ( 32767, 2                   ),
                HID_REPORT_COUNT  ( 2                          ),
                HID_REPORT_SIZE   ( 16                         ),
                HID_INPUT         ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),
                HID_USAGE         ( HID_USAGE_DESKTOP_WHEEL    ),
                HID_LOGICAL_MIN   ( 0x81                       ),
                HID_LOGICAL_MAX   ( 0x7f                       ),
                HID_REPORT_COUNT  ( 1                          ),
                HID_REPORT_SIZE   ( 8                          ),
                HID_INPUT         ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),
            HID_USAGE_PAGE   ( HID_USAGE_PAGE_CONSUMER         ),
                HID_USAGE_N       ( HID_USAGE_CONSUMER_AC_PAN, 2 ),
                HID_LOGICAL_MIN   ( 0x81                       ),
                HID_LOGICAL_MAX   ( 0x7f                       ),
                HID_REPORT_COUNT  ( 1                          ),
                HID_REPORT_SIZE   ( 8                          ),
                HID_INPUT         ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),
        HID_COLLECTION_END,
    HID_COLLECTION_END,
    HID_USAGE_PAGE_N ( 0xff00, 2                               ),
    HID_USAGE        ( 0x01                                    ),
    HID_COLLECTION   ( HID_COLLECTION_APPLICATION              ),
        HID_REPORT_ID    ( REPORT_ID_CONFIG                    )
        HID_LOGICAL_MIN  ( 0                                   ),
        HID_LOGICAL_MAX_N( 255, 2                              ),
        HID_REPORT_SIZE  ( 8                                   ),
        HID_REPORT_COUNT ( HID_CONFIG_REPORT_SIZE              ),
        HID_USAGE        ( 0x02                                ),
        HID_OUTPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
        HID_USAGE        ( 0x03                                ),
        HID_FEATURE      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
    HID_COLLECTION_END,
};
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + \
                          TUD_CDC_DESC_LEN + \
                          TUD_HID_DESC_LEN + \
                          TUD_AUDIO_DEVICE_DESC_LEN)

const uint8_t dongle_usb_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 250),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, STRID_HID, HID_ITF_PROTOCOL_NONE,
                       sizeof(dongle_hid_report_descriptor), EPNUM_HID_IN, 16, 1),
    TUD_AUDIO_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, STRID_UAC, 0x00, EPNUM_AUDIO_IN, 0x00),
};

const char *dongle_usb_string_descriptor[] = {
    (const char[]){0x09, 0x04},
    "Audio QoS",
    "Audio QoS",
    "DONGLE-4118",
    "Audio QoS CDC",
    "Audio QoS HID",
    "Audio QoS Microphone",
    "Audio QoS Mic Stream",
};

const int dongle_usb_string_descriptor_count =
    sizeof(dongle_usb_string_descriptor) / sizeof(dongle_usb_string_descriptor[0]);

const int dongle_usb_uac_mic_interface = ITF_NUM_AUDIO_STREAMING_MIC;
