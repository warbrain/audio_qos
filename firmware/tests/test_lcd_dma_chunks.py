"""Execute production LCD DMA helpers with SPI/time stubs; never a panel benchmark."""
from pathlib import Path
import re
from test_audio_recovery_042 import _build_host_executable, _run

ROOT = Path(__file__).resolve().parents[1]
LCD = ROOT / "scepter/main/lcd"


def _function(source: str, name: str) -> str:
    """Compile the real function, rather than reimplementing its algorithm."""
    match = re.search(r"^(?:static\s+(?:inline\s+)?)?(?:void|bool|esp_err_t)\s+"
                      + re.escape(name) + r"\s*\(", source, re.MULTILINE)
    assert match is not None, name
    opening = source.index("{", match.end())
    depth = 1
    cursor = opening + 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[match.start():cursor]


def test_persistent_dma_chunks_and_failure_recovery(tmp_path: Path) -> None:
    source = (LCD / "lcd_driver.c").read_text(encoding="utf-8")
    policy = source[source.index("/* Pixel failures pause"):
                    source.index("/* ── Low-level SPI helpers")]
    functions = "\n\n".join(_function(source, name) for name in (
        "lcd_write_cmd_ex", "lcd_write_data_ex", "lcd_write_data",
        "lcd_write_stream", "write_reg", "lcd_set_address_window_impl", "lcd_set_address_window",
        "lcd_write_pixels", "lcd_write_frame", "lcd_draw_bitmap_locked", "lcd_draw_bitmap",
        "lcd_fill_rect_locked", "lcd_fill_rect", "lcd_draw_pixel_locked", "lcd_draw_pixel",
    ))
    commands = "\n".join(re.findall(r"^#define LCD_CMD_\w+\s+.+$", source, re.MULTILINE))
    stubs = tmp_path / "stubs"
    stubs.mkdir()
    (stubs / "esp_err.h").write_text("""#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE -2
#define ESP_ERR_NO_MEM -3
#define ESP_ERR_INVALID_SIZE -4
""", encoding="utf-8")

    harness = tmp_path / "lcd_actual_helpers.c"
    preamble = r"""
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lcd_driver.h"
typedef uint32_t TickType_t;
typedef void *spi_device_handle_t;
typedef void *SemaphoreHandle_t;
typedef struct { size_t length; const void *tx_buffer; void *user; unsigned flags; } spi_transaction_t;
#define SPI_TRANS_CS_KEEP_ACTIVE 1U
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
static const char *TAG = "lcd";
static spi_device_handle_t spi_handle = (void *)1;
static SemaphoreHandle_t spi_mutex = (void *)2, operation_mutex = (void *)3;
static bool lcd_suspended, bus_acquired;
static uint16_t *fb_buffer;
static int fb_x, fb_y, fb_w, fb_h;
static const size_t DMA_BUF_SIZE = LCD_MAX_TRANSFER_BYTES;
_Alignas(4) static uint8_t dma_storage[LCD_MAX_TRANSFER_BYTES];
static uint8_t *dma_buffer = dma_storage;
static TickType_t now_tick;
static unsigned spi_lock, operation_lock, acquires, releases, attempts, data_attempts, log_count;
static unsigned fail_attempt, fail_data_attempt;
static bool acquire_failure, require_operation_guard;
static uint8_t wire[500000], command_wire[2000];
static size_t wire_size, command_size;
static size_t successful_lengths[1000];
static unsigned successful_flags[1000], successful_data_count;
static TickType_t xTaskGetTickCount(void) { return now_tick; }
static const char *esp_err_to_name(esp_err_t e) { (void)e; return "stub"; }
static void test_error_log(const char *tag,const char *format,...) { (void)tag; (void)format; ++log_count; }
#define ESP_LOGE(tag,...) test_error_log(tag,__VA_ARGS__)
static void xSemaphoreTake(void *mutex, uint32_t wait) {
    assert(wait==portMAX_DELAY);
    if(mutex==spi_mutex) { assert(spi_lock==0); ++spi_lock; }
    else { assert(mutex==operation_mutex && operation_lock==0); ++operation_lock; }
}
static void xSemaphoreGive(void *mutex) {
    if(mutex==spi_mutex) { assert(spi_lock==1); --spi_lock; }
    else { assert(mutex==operation_mutex && operation_lock==1); --operation_lock; }
}
static esp_err_t spi_device_acquire_bus(void *handle,uint32_t wait) {
    assert(handle==spi_handle && wait==portMAX_DELAY && spi_lock==1);
    ++acquires; return acquire_failure ? ESP_FAIL : ESP_OK;
}
static void spi_device_release_bus(void *handle) {
    assert(handle==spi_handle && spi_lock==1 && bus_acquired); ++releases;
}
static esp_err_t spi_device_transmit(void *handle,spi_transaction_t *t) {
    assert(handle==spi_handle && spi_lock==1);
    if(require_operation_guard) assert(operation_lock==1);
    assert(!(t->flags & SPI_TRANS_CS_KEEP_ACTIVE) || bus_acquired);
    ++attempts;
    if (t->user==(void *)1) {
        ++data_attempts;
        assert(t->tx_buffer==dma_buffer);
        assert(((uintptr_t)t->tx_buffer & 3U)==0);
        assert(t->length % 8U==0 && t->length/8U<=DMA_BUF_SIZE);
        /* Local IDF S3 GDMA internal TX alignment is one byte. Keep the
         * original length of short registers and final pixels; no padding. */
    } else { assert(t->user==NULL && t->length==8U); }
    if ((fail_attempt && attempts==fail_attempt) ||
        (fail_data_attempt && data_attempts==fail_data_attempt)) return ESP_ERR_NO_MEM;
    if(t->user==(void *)1) {
        size_t bytes=t->length/8U; assert(wire_size+bytes<=sizeof(wire));
        memcpy(wire+wire_size,t->tx_buffer,bytes); wire_size+=bytes;
        successful_lengths[successful_data_count]=bytes;
        successful_flags[successful_data_count++]=t->flags;
    } else {
        assert(command_size<sizeof(command_wire));
        command_wire[command_size++]=*(const uint8_t *)t->tx_buffer;
    }
    return ESP_OK;
}
"""
    cases = r"""
static void reset_test(void) {
    assert(spi_lock==0 && operation_lock==0);
    spi_handle=(void *)1; lcd_suspended=false; bus_acquired=false;
    dma_buffer=dma_storage; fb_buffer=NULL; now_tick=0;
    spi_error_retry_pending=false; spi_error_log_valid=false;
    acquires=releases=attempts=data_attempts=log_count=0;
    fail_attempt=fail_data_attempt=0; acquire_failure=false; require_operation_guard=false;
    wire_size=command_size=successful_data_count=0;
}
static void pattern(uint8_t *buffer,size_t size,unsigned salt) {
    for(size_t i=0;i<size;i++) buffer[i]=(uint8_t)(i*37U+salt);
}
static void expect_window(void) {
    assert(command_size==3U && command_wire[0]==LCD_CMD_CASET &&
           command_wire[1]==LCD_CMD_RASET && command_wire[2]==LCD_CMD_RAMWR);
    const uint8_t params[8]={0,34,0,205,0,0,1,63};
    assert(wire_size>=8U && memcmp(wire,params,8)==0);
}
int main(void) {
    static uint8_t data[LCD_MAX_TRANSFER_BYTES*3U+4U];
    pattern(data,sizeof(data),9);
    const size_t lengths[]={1U,2U,4U,5U,14U,LCD_MAX_TRANSFER_BYTES-4U,
        LCD_MAX_TRANSFER_BYTES,LCD_MAX_TRANSFER_BYTES+1U,
        LCD_MAX_TRANSFER_BYTES+2U,LCD_MAX_TRANSFER_BYTES+3U,
        LCD_MAX_TRANSFER_BYTES*3U+4U};
    for(unsigned k=0;k<sizeof(lengths)/sizeof(lengths[0]);k++) {
        reset_test(); size_t len=lengths[k];
        assert(lcd_write_stream(data,len)==ESP_OK);
        assert(wire_size==len && memcmp(wire,data,len)==0);
        unsigned chunks=(unsigned)((len+DMA_BUF_SIZE-1U)/DMA_BUF_SIZE);
        assert(successful_data_count==chunks);
        for(unsigned i=0;i<chunks;i++)
            assert(successful_flags[i]==(i+1U<chunks ? SPI_TRANS_CS_KEEP_ACTIVE : 0U));
        assert(acquires==(unsigned)(chunks>1U) && releases==(unsigned)(chunks>1U) && !bus_acquired);
    }
    puts("PASS byte ordering small large exact and two-byte final chunk");

    reset_test(); assert(lcd_write_stream(NULL,4)==ESP_OK && attempts==0);
    assert(lcd_write_stream(data,0)==ESP_OK && attempts==0);
    lcd_suspended=true; assert(lcd_write_stream(data,4)==ESP_OK && attempts==0);
    lcd_suspended=false; spi_handle=NULL; assert(lcd_write_stream(data,4)==ESP_OK && attempts==0);
    puts("PASS empty suspended and absent device preserve no-op behavior");

    reset_test(); acquire_failure=true;
    assert(lcd_write_stream(data,sizeof(data))==ESP_FAIL);
    assert(attempts==0 && acquires==1 && releases==0 && log_count==1 && !bus_acquired);
    puts("PASS bus-acquire failure has no transfer or leaked ownership");

    reset_test(); fail_data_attempt=2;
    assert(lcd_write_stream(data,sizeof(data))==ESP_ERR_NO_MEM);
    assert(data_attempts==2 && wire_size==DMA_BUF_SIZE);
    assert(memcmp(wire,data,DMA_BUF_SIZE)==0 && releases==1 && !bus_acquired);
    unsigned count=attempts;
    assert(lcd_write_stream(data,sizeof(data))==ESP_ERR_INVALID_STATE && attempts==count);
    now_tick=99; assert(!lcd_pixel_retry_ready());
    now_tick=100; assert(lcd_pixel_retry_ready());
    fail_data_attempt=0;
    size_t previous=wire_size;
    assert(lcd_write_stream(data,sizeof(data))==ESP_OK);
    assert(wire_size==previous+sizeof(data) && memcmp(wire+previous,data,sizeof(data))==0);
    puts("PASS first failed chunk aborts frame no automatic replay bounded retry");

    reset_test(); fail_data_attempt=2;
    assert(lcd_write_stream(data,DMA_BUF_SIZE+4U)==ESP_ERR_NO_MEM);
    assert(releases==1 && !bus_acquired && successful_data_count==1);
    puts("PASS last-chunk failure releases acquired bus");

    reset_test(); dma_buffer=NULL;
    assert(lcd_write_data_ex(data,4,false)==ESP_ERR_NO_MEM && attempts==0);
    assert(log_count==1);
    dma_buffer=dma_storage;
    assert(lcd_write_data_ex(data,DMA_BUF_SIZE+1U,false)==ESP_ERR_INVALID_SIZE && attempts==0);
    assert(log_count==1);
    puts("PASS missing persistent buffer and oversized private call fail without SPI");

    reset_test();
    for(unsigned frame=0;frame<25U;frame++) {
        now_tick=frame*100U; fail_data_attempt=data_attempts+1U;
        assert(lcd_write_stream(data,sizeof(data))==ESP_ERR_NO_MEM);
    }
    assert(data_attempts==25U && log_count==3U && wire_size==0 && acquires==releases);
    puts("PASS repeated failure one attempt per frame logs at most once per second");

    reset_test(); now_tick=UINT32_MAX-49U; fail_data_attempt=1;
    assert(lcd_write_stream(data,4)==ESP_ERR_NO_MEM);
    now_tick=49U; assert(!lcd_pixel_retry_ready());
    now_tick=50U; assert(lcd_pixel_retry_ready());
    puts("PASS retry arithmetic survives tick wrap");

    reset_test();
    static uint8_t frame[LCD_WIDTH*LCD_HEIGHT*2U]; pattern(frame,sizeof(frame),19);
    require_operation_guard=true;
    lcd_write_frame(0,0,LCD_WIDTH-1,LCD_HEIGHT-1,frame,sizeof(frame));
    expect_window();
    assert(wire_size==8U+sizeof(frame) && memcmp(wire+8,frame,sizeof(frame))==0);
    assert(data_attempts==2U+(sizeof(frame)+DMA_BUF_SIZE-1U)/DMA_BUF_SIZE &&
           acquires==1U && releases==1U && operation_lock==0);
    puts("PASS full frame exact window bytes DC and CS ordering");

    reset_test(); fail_data_attempt=4;
    require_operation_guard=true;
    lcd_write_frame(0,0,LCD_WIDTH-1,LCD_HEIGHT-1,frame,sizeof(frame));
    assert(command_size==3U && data_attempts==4U && wire_size==8U+DMA_BUF_SIZE);
    assert(memcmp(wire+8,frame,DMA_BUF_SIZE)==0 && acquires==releases);
    count=attempts;
    require_operation_guard=true;
    lcd_write_frame(0,0,LCD_WIDTH-1,LCD_HEIGHT-1,frame,sizeof(frame));
    assert(attempts==count);
    now_tick=100; fail_data_attempt=0;
    previous=wire_size;
    require_operation_guard=true;
    lcd_write_frame(0,0,LCD_WIDTH-1,LCD_HEIGHT-1,frame,sizeof(frame));
    assert(command_size==6U && command_wire[3]==LCD_CMD_CASET && command_wire[5]==LCD_CMD_RAMWR);
    assert(wire_size==previous+8U+sizeof(frame));
    assert(memcmp(wire+previous+8,frame,sizeof(frame))==0);
    assert(operation_lock==0 && spi_lock==0 && !bus_acquired);
    puts("PASS failed frame stops and next full frame resets address and starts at byte zero");

    reset_test(); fail_attempt=1;
    require_operation_guard=true;
    lcd_write_frame(0,0,LCD_WIDTH-1,LCD_HEIGHT-1,frame,sizeof(frame));
    assert(attempts==1U && wire_size==0 && acquires==releases);
    puts("PASS failed address command stops remaining window and pixel transfers");

    reset_test();
    require_operation_guard=true;
    lcd_fill_rect(0,0,LCD_WIDTH,65,0x1234U);
    assert(wire_size==8U+LCD_WIDTH*65U*2U);
    for(size_t i=8;i<wire_size;i+=2U) assert(wire[i]==0x12 && wire[i+1U]==0x34);
    assert(successful_flags[successful_data_count-1U]==0U && acquires==releases);
    puts("PASS fill RGB565 endian and chunk border");

    reset_test();
    static uint16_t bitmap[LCD_WIDTH*65U];
    for(size_t i=0;i<LCD_WIDTH*65U;i++) bitmap[i]=(uint16_t)(i+0x4567U);
    require_operation_guard=true;
    lcd_draw_bitmap(0,0,LCD_WIDTH,65,bitmap);
    assert(wire_size==8U+sizeof(bitmap));
    for(size_t i=0;i<LCD_WIDTH*65U;i++)
        assert(wire[8U+i*2U]==(uint8_t)(bitmap[i]>>8) &&
               wire[9U+i*2U]==(uint8_t)bitmap[i]);
    assert(acquires==releases);
    puts("PASS bitmap RGB565 endian and chunk border");

    reset_test(); fail_data_attempt=4;
    require_operation_guard=true;
    lcd_fill_rect(0,0,LCD_WIDTH,65,0x1234U);
    assert(data_attempts==4U && acquires==releases && log_count==1U);
    reset_test(); fail_data_attempt=4;
    require_operation_guard=true;
    lcd_draw_bitmap(0,0,LCD_WIDTH,65,bitmap);
    assert(data_attempts==4U && acquires==releases && log_count==1U);
    puts("PASS fill and bitmap abort their failed frame without per-chunk retry");

    reset_test();
    uint16_t framebuffer[4]={0};
    fb_buffer=framebuffer; fb_x=fb_y=0; fb_w=fb_h=2;
    spi_error_retry_pending=true;
    lcd_fill_rect(0,0,2,2,0xBEEF);
    lcd_draw_pixel(0,0,0xFACE);
    assert(attempts==0 && framebuffer[0]==0xFACE && framebuffer[3]==0xBEEF);
    puts("PASS in-memory framebuffer remains editable during pixel backoff");
    assert(spi_lock==0 && operation_lock==0);
    puts("LCD actual DMA helper checks passed (15 scenarios)");
    return 0;
}
"""
    harness.write_text(preamble + commands + "\n" + policy + "\n" + functions + "\n" + cases,
                       encoding="utf-8")
    executable, env = _build_host_executable([harness], tmp_path,
                                             extra_includes=(stubs, LCD))
    result = _run([str(executable)], cwd=tmp_path, env=env)
    assert "LCD actual DMA helper checks passed (15 scenarios)" in result.stdout
