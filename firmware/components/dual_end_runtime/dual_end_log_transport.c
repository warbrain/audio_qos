#include "dual_end_log_transport.h"
#include "sdkconfig.h"
#include <stdatomic.h>

static atomic_bool s_enabled;
static atomic_uint s_queue_drop,s_format_drop,s_short_write,s_backend_error,s_queue_highwater;
static atomic_uint s_format_drop_long,s_format_drop_chars;

void dual_end_log_transport_snapshot(dual_end_log_transport_stats_t *out)
{
    if(!out)return;
    *out=(dual_end_log_transport_stats_t){
        .enabled=atomic_load_explicit(&s_enabled,memory_order_acquire),
        .queue_drop=atomic_load_explicit(&s_queue_drop,memory_order_relaxed),
        .format_drop=atomic_load_explicit(&s_format_drop,memory_order_relaxed),
        .format_drop_long=atomic_load_explicit(&s_format_drop_long,memory_order_relaxed),
        .format_drop_chars=atomic_load_explicit(&s_format_drop_chars,memory_order_relaxed),
        .short_write=atomic_load_explicit(&s_short_write,memory_order_relaxed),
        .backend_error=atomic_load_explicit(&s_backend_error,memory_order_relaxed),
        .queue_highwater=atomic_load_explicit(&s_queue_highwater,memory_order_relaxed),
    };
}

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG && CONFIG_LOG_VERSION == 1
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define LOG_TRANSPORT_STACK_BYTES 4096U
#define LOG_TRANSPORT_PRIORITY 1U
#define LOG_TRANSPORT_WRITE_WAIT_MS 100U
#define LOG_TRANSPORT_WRITE_ATTEMPTS 3U
typedef struct {
    uint16_t length;
    char text[DUAL_END_LOG_TRANSPORT_LINE_BYTES];
} log_record_t;
/* Control stays internal; only task-accessed record storage may live in PSRAM.
 * No ISR/flash-cache-disabled log path is redirected by normal LOG_VERSION1. */
static StaticQueue_t s_queue_control;
static QueueHandle_t s_queue;
static bool s_attempted,s_needs_boundary;

static void counter_increment(atomic_uint *counter)
{
    unsigned value=atomic_load_explicit(counter,memory_order_relaxed);
    while(value!=UINT32_MAX && !atomic_compare_exchange_weak_explicit(counter,&value,value+1U,
        memory_order_relaxed,memory_order_relaxed)) {}
}
static int log_transport_enqueue(const char *format,va_list args)
{
    log_record_t record={0};
    if(!format) {counter_increment(&s_format_drop);counter_increment(&s_format_drop_chars);return 0;}
    const int length=vsnprintf(record.text,sizeof(record.text),format,args);
    if(length<=0) {
        counter_increment(&s_format_drop);
        counter_increment(&s_format_drop_chars);
        return length;
    }
    if((size_t)length>=sizeof(record.text)) {
        counter_increment(&s_format_drop);
        counter_increment(&s_format_drop_long);
        return length;
    }
    size_t bytes=(size_t)length;
    /* Treat each callback as one complete LOG_VERSION1 entry. Reject embedded
     * NUL/multiple lines and overlength data instead of leaking a JSON prefix.
     * Raw one-line esp_log_write calls without a newline gain a final LF. */
    if(memchr(record.text,'\0',bytes) ||
        memchr(record.text,'\n',bytes-(record.text[bytes-1]=='\n'?1U:0U))) {
        counter_increment(&s_format_drop);
        counter_increment(&s_format_drop_chars);
        return length;
    }
    if(record.text[bytes-1]!='\n') {
        if(bytes+1U>=sizeof(record.text)) {
            counter_increment(&s_format_drop);
            counter_increment(&s_format_drop_long);
            return length;
        }
        record.text[bytes++]='\n';record.text[bytes]='\0';
    }
    record.length=(uint16_t)bytes;
    if(xQueueSend(s_queue,&record,0)!=pdTRUE) counter_increment(&s_queue_drop);
    else {
        unsigned used=(unsigned)uxQueueMessagesWaiting(s_queue);
        unsigned high=atomic_load_explicit(&s_queue_highwater,memory_order_relaxed);
        while(used>high && !atomic_compare_exchange_weak_explicit(&s_queue_highwater,&high,used,
            memory_order_relaxed,memory_order_relaxed)) {}
    }
    return length;
}
/* The IDF ringbuffer write is all-or-zero. Keep defensive partial-write
 * handling bounded, and never concatenate the following entry onto a fragment
 * if a future/backend fault violates that contract. Worker is the sole owner. */
static bool log_transport_boundary(void)
{
    const int written=usb_serial_jtag_write_bytes("\n",1,pdMS_TO_TICKS(LOG_TRANSPORT_WRITE_WAIT_MS));
    if(written==1) {s_needs_boundary=false;return true;}
    counter_increment(&s_short_write);return false;
}
static void log_transport_write(const log_record_t *record)
{
    flockfile(stdout);
    if(!usb_serial_jtag_is_driver_installed()) {
        counter_increment(&s_backend_error);funlockfile(stdout);return;
    }
    if(s_needs_boundary && !log_transport_boundary()) {
        counter_increment(&s_backend_error);funlockfile(stdout);return;
    }
    size_t offset=0;
    for(unsigned attempt=0;attempt<LOG_TRANSPORT_WRITE_ATTEMPTS && offset<record->length;++attempt) {
        size_t remaining=record->length-offset;
        int written=usb_serial_jtag_write_bytes(record->text+offset,remaining,
            pdMS_TO_TICKS(LOG_TRANSPORT_WRITE_WAIT_MS));
        if(written==(int)remaining) {offset+=remaining;break;}
        counter_increment(&s_short_write);
        if(written<0 || (size_t)written>remaining)break;
        offset+=(size_t)written;
    }
    if(offset!=record->length) {
        counter_increment(&s_backend_error);
        if(offset) {
            s_needs_boundary=true;
            (void)log_transport_boundary();
        }
    }
    funlockfile(stdout);
}
static void log_transport_worker(void *arg)
{
    (void)arg;
    (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    log_record_t record;
    for(;;) {
        if(xQueueReceive(s_queue,&record,portMAX_DELAY)==pdTRUE)log_transport_write(&record);
    }
}
esp_err_t dual_end_log_transport_start(scepter_role_t role)
{
    if(role!=SCEPTER_ROLE_SCEPTER)return ESP_OK;
    if(s_attempted)return atomic_load_explicit(&s_enabled,memory_order_acquire)?ESP_OK:ESP_ERR_INVALID_STATE;
    s_attempted=true;
    uint32_t storage_caps=MALLOC_CAP_8BIT;
#if CONFIG_SPIRAM
    storage_caps|=MALLOC_CAP_SPIRAM;
#else
    storage_caps|=MALLOC_CAP_INTERNAL;
#endif
    uint8_t *storage=heap_caps_malloc(DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH*sizeof(log_record_t),storage_caps);
    if(!storage)return ESP_ERR_NO_MEM;
    s_queue=xQueueCreateStatic(DUAL_END_LOG_TRANSPORT_QUEUE_DEPTH,sizeof(log_record_t),storage,&s_queue_control);
    if(!s_queue) {heap_caps_free(storage);return ESP_ERR_NO_MEM;}
    TaskHandle_t worker=NULL;
#if CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    BaseType_t created=xTaskCreatePinnedToCoreWithCaps(log_transport_worker,"usb_log_tx",
        LOG_TRANSPORT_STACK_BYTES,NULL,LOG_TRANSPORT_PRIORITY,&worker,0,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
#else
    BaseType_t created=xTaskCreatePinnedToCore(log_transport_worker,"usb_log_tx",
        LOG_TRANSPORT_STACK_BYTES,NULL,LOG_TRANSPORT_PRIORITY,&worker,0);
#endif
    if(created!=pdPASS) {
        vQueueDelete(s_queue);s_queue=NULL;heap_caps_free(storage);return ESP_ERR_NO_MEM;
    }
    /* Pairing normally installed the driver already: preserve its RX ring and
     * task, and never uninstall another owner's driver on an optional failure. */
    esp_err_t result=ESP_OK;
    if(!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config={.tx_buffer_size=2048,.rx_buffer_size=256};
        result=usb_serial_jtag_driver_install(&config);
    }
    if(result!=ESP_OK) {
        counter_increment(&s_backend_error);
#if CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
        vTaskDeleteWithCaps(worker);
#else
        vTaskDelete(worker);
#endif
        vQueueDelete(s_queue);s_queue=NULL;heap_caps_free(storage);return result;
    }
    /* This changes no fd flags/read ownership. Existing command stdio keeps
     * its nonblocking flags; only the ready worker may wait for USB output. */
    usb_serial_jtag_vfs_use_driver();
    atomic_store_explicit(&s_enabled,true,memory_order_release);
    (void)esp_log_set_vprintf(log_transport_enqueue);
    xTaskNotifyGive(worker);
    return ESP_OK;
}
#else
esp_err_t dual_end_log_transport_start(scepter_role_t role)
{
    (void)role;
    /* Leave UART, Dongle CDC mirroring and LOG_VERSION2 formatting untouched. */
    return ESP_OK;
}
#endif
