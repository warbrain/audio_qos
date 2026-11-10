#pragma once

#include "esp_assert.h"
#include "sdkconfig.h"
#include "tusb_option.h"
#include "uac_config.h"
#include "uac_descriptors.h"
#include "tusb_config_uac.h"

#ifndef CONFIG_TINYUSB_CDC_COUNT
#define CONFIG_TINYUSB_CDC_COUNT 0
#endif

#ifndef CONFIG_TINYUSB_HID_COUNT
#define CONFIG_TINYUSB_HID_COUNT 0
#endif

#ifndef CONFIG_TINYUSB_DEBUG_LEVEL
#define CONFIG_TINYUSB_DEBUG_LEVEL 0
#endif

#define CFG_TUD_ENABLED                 1
#define CFG_TUD_MAX_SPEED               OPT_MODE_FULL_SPEED
#define CFG_TUSB_RHPORT0_MODE           (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS                     OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG                  CONFIG_TINYUSB_DEBUG_LEVEL
#define CFG_TUSB_DEBUG_PRINTF           esp_rom_printf

#ifdef CONFIG_TINYUSB_MODE_DMA
#define CFG_TUD_DWC2_DMA_ENABLE         1
#endif
#define CFG_TUD_DWC2_SLAVE_ENABLE       1

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN              TU_ATTR_ALIGNED(4)
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE          64
#endif

#define CFG_TUD_CDC                     CONFIG_TINYUSB_CDC_COUNT
#define CFG_TUD_HID                     CONFIG_TINYUSB_HID_COUNT
#define CFG_TUD_AUDIO                   1
#define CFG_TUD_MSC                     0
#define CFG_TUD_MIDI                    0
#define CFG_TUD_VENDOR                  0
#define CFG_TUD_ECM_RNDIS               0
#define CFG_TUD_NCM                     0
#define CFG_TUD_DFU                     0
#define CFG_TUD_DFU_RUNTIME             0
#define CFG_TUD_BTH                     0

#define CFG_TUD_CDC_RX_BUFSIZE          CONFIG_TINYUSB_CDC_RX_BUFSIZE
#define CFG_TUD_CDC_TX_BUFSIZE          CONFIG_TINYUSB_CDC_TX_BUFSIZE
#define CFG_TUD_CDC_EP_BUFSIZE          CONFIG_TINYUSB_CDC_EP_BUFSIZE

#define CFG_TUD_MSC_BUFSIZE             512
#define CFG_TUD_MIDI_EP_BUFSIZE         64
#define CFG_TUD_MIDI_EPSIZE             CFG_TUD_MIDI_EP_BUFSIZE
#define CFG_TUD_MIDI_RX_BUFSIZE         64
#define CFG_TUD_MIDI_TX_BUFSIZE         64
#define CFG_TUD_VENDOR_RX_BUFSIZE       64
#define CFG_TUD_VENDOR_TX_BUFSIZE       64
#define CFG_TUD_VENDOR_EPSIZE           64
#define CFG_TUD_DFU_XFER_BUFSIZE        64
#define CFG_TUD_BTH_ISO_ALT_COUNT       0
