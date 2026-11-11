/*
 * LCD Driver for ST7789 (vendor-confirmed controller)
 * Resolution: 172(H) x 320(V), RGB565, 4-wire SPI
 * Target: ESP32-S3 with ESP-IDF v6.0.2
 *
 * The ST7789 has an internal 240x320 GRAM. The visible 172x320 window is
 * centered with an X offset of 34 pixels (LCD_X_GAP).
 */

#include "lcd_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "string.h"

/* ── Pin mapping (from HARDWARE_PINOUT.md) ─────────────────────────── */
#define PIN_LCD_SCL     GPIO_NUM_1
#define PIN_LCD_SDA     GPIO_NUM_2
#define PIN_LCD_CS      GPIO_NUM_39
#define PIN_LCD_DC      GPIO_NUM_40
#define PIN_LCD_RST     GPIO_NUM_41
#define PIN_LCD_BL      GPIO_NUM_38

/* ── SPI configuration ──────────────────────────────────────────────── */
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_SPI_FREQ         (20 * 1000 * 1000)  /* Balanced animation rate and signal margin */

/* DMA transfer buffer size: enough for 64 lines of RGB565 at a time */
#define LCD_DMA_BUF_SIZE     LCD_MAX_TRANSFER_BYTES

/* ── ST7789 / MIPI DCS command definitions ──────────────────────────── */
#define LCD_CMD_SWRESET      0x01
#define LCD_CMD_SLPIN        0x10
#define LCD_CMD_SLPOUT       0x11
#define LCD_CMD_PTLON        0x12
#define LCD_CMD_NORON        0x13
#define LCD_CMD_INVOFF       0x20
#define LCD_CMD_INVON        0x21
#define LCD_CMD_DISPOFF      0x28
#define LCD_CMD_DISPON       0x29
#define LCD_CMD_CASET        0x2A
#define LCD_CMD_RASET        0x2B
#define LCD_CMD_RAMWR        0x2C
#define LCD_CMD_MADCTL       0x36
#define LCD_CMD_IDMOFF       0x38
#define LCD_CMD_IDMON        0x39
#define LCD_CMD_COLMOD       0x3A
#define LCD_CMD_WRDISBV      0x51
#define LCD_CMD_RDDISBV      0x52
#define LCD_CMD_WRCTRLD      0x53
#define LCD_CMD_TEARON       0x35
#define LCD_CMD_TEARLINE     0x44

/* ── ST7789 extended command definitions ─────────────────────────────── */
#define LCD_CMD_PORCTRL      0xB2
#define LCD_CMD_GCTRL        0xB7
#define LCD_CMD_VCOMS        0xBB
#define LCD_CMD_LCMCTRL      0xC0
#define LCD_CMD_VDVVRHEN     0xC2
#define LCD_CMD_VRHS         0xC3
#define LCD_CMD_VDVS         0xC4
#define LCD_CMD_FRCTRL2      0xC6
#define LCD_CMD_PWCTRL1      0xD0
#define LCD_CMD_PVGAMCTRL    0xE0
#define LCD_CMD_NVGAMCTRL    0xE1

/* ── Module globals ─────────────────────────────────────────────────── */
static const char *TAG = "lcd";
static spi_device_handle_t spi_handle = NULL;
static volatile bool lcd_suspended = false;

/* In-memory framebuffer hook for flicker-free rendering */
static uint16_t *fb_buffer = NULL;
static int fb_x = 0, fb_y = 0, fb_w = 0, fb_h = 0;

/* Mutex to protect SPI transactions from concurrent tasks */
static SemaphoreHandle_t spi_mutex = NULL;

/*
 * Serializes complete LCD operations. In particular, a frame's
 * CASET/RASET/RAMWR/data sequence must never be interleaved with DISPON or the
 * ST7789 resume initialization sequence.
 */
static SemaphoreHandle_t operation_mutex = NULL;

/*
 * CS-keep-active is only legal while the SPI bus is acquired.
 * This flag tracks whether the current frame burst has acquired it.
 */
static bool bus_acquired = false;

/*
 * Single reusable DMA-capable buffer for fill operations.
 * Allocated once at init to avoid repeated heap churn.
 */
static uint8_t *dma_buffer = NULL;
static const size_t DMA_BUF_SIZE = LCD_WIDTH * 24 * 2; /* 24 lines at a time */

/* Pixel failures pause new frames without blocking the task or flooding USB
 * logs. The animation task retains its existing frame pacing; the next full
 * frame re-establishes CASET/RASET/RAMWR and restarts from pixel zero. */
#define LCD_SPI_ERROR_RETRY_MS 100U
#define LCD_SPI_ERROR_LOG_INTERVAL_MS 1000U
static bool spi_error_retry_pending;
static TickType_t spi_error_retry_tick;
static bool spi_error_log_valid;
static TickType_t spi_error_log_tick;

static bool lcd_pixel_retry_ready(void)
{
    if (!spi_error_retry_pending) return true;
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - spi_error_retry_tick) < pdMS_TO_TICKS(LCD_SPI_ERROR_RETRY_MS)) {
        return false;
    }
    spi_error_retry_pending = false;
    return true;
}

static void lcd_note_spi_failure(const char *operation, esp_err_t result)
{
    TickType_t now = xTaskGetTickCount();
    spi_error_retry_pending = true;
    spi_error_retry_tick = now;
    if (!spi_error_log_valid ||
        (TickType_t)(now - spi_error_log_tick) >= pdMS_TO_TICKS(LCD_SPI_ERROR_LOG_INTERVAL_MS)) {
        ESP_LOGE(TAG, "%s failed: %s (pixel retry delayed)", operation, esp_err_to_name(result));
        spi_error_log_tick = now;
        spi_error_log_valid = true;
    }
}

/* Caller holds spi_mutex. Also release a kept-CS burst after any SPI error. */
static void lcd_release_bus_locked(void)
{
    if (bus_acquired) {
        spi_device_release_bus(spi_handle);
        bus_acquired = false;
    }
}

/* ── Low-level SPI helpers ──────────────────────────────────────────── */

/**
 * @brief SPI pre-transfer callback: assert DC pin level.
 *
 * Used by the ESP-IDF SPI driver to toggle DC automatically before each
 * transaction.  user = 0 means command (DC low), user = 1 means data (DC high).
 */
static void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level(PIN_LCD_DC, (int)t->user);
}

/**
 * @brief Write a command byte (DC = 0)
 * @param keep_active  true = keep CS asserted after this transaction
 *
 * Uses spi_device_transmit (blocking queued API) because
 * SPI_TRANS_CS_KEEP_ACTIVE is not allowed with polling_transmit.
 */
static esp_err_t lcd_write_cmd_ex(uint8_t cmd, bool keep_active)
{
    if (spi_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (spi_mutex != NULL) xSemaphoreTake(spi_mutex, portMAX_DELAY);

    /* SPI_TRANS_CS_KEEP_ACTIVE requires the bus to be acquired first. */
    if (keep_active && !bus_acquired) {
        esp_err_t r = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
        if (r != ESP_OK) {
            lcd_note_spi_failure("SPI bus acquire", r);
            if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
            return r;
        }
        bus_acquired = true;
    }

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void *)0,   /* DC = 0: command */
        .flags = keep_active ? SPI_TRANS_CS_KEEP_ACTIVE : 0,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        lcd_note_spi_failure("SPI command write", ret);
    }

    /* Release after the last transaction, including a failed kept-CS burst. */
    if (!keep_active || ret != ESP_OK) {
        lcd_release_bus_locked();
    }

    if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
    return ret;
}

static inline void lcd_write_cmd(uint8_t cmd)
{
    if (lcd_suspended) return;
    lcd_write_cmd_ex(cmd, false);
}

/**
 * @brief Write multiple data bytes (DC = 1)
 * @param keep_active  true = keep CS asserted after this transaction
 *
 * Uses spi_device_transmit so CS can be kept active across chunks.
 */
static esp_err_t lcd_write_data_ex(const uint8_t *data, size_t len, bool keep_active)
{
    if (len == 0 || data == NULL) return ESP_OK;
    if (spi_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (spi_mutex != NULL) xSemaphoreTake(spi_mutex, portMAX_DELAY);

    if (dma_buffer == NULL || len > DMA_BUF_SIZE) {
        esp_err_t result = dma_buffer == NULL ? ESP_ERR_NO_MEM : ESP_ERR_INVALID_SIZE;
        lcd_note_spi_failure("SPI data buffer", result);
        lcd_release_bus_locked();
        if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
        return result;
    }

    /* SPI_TRANS_CS_KEEP_ACTIVE requires the bus to be acquired first. */
    if (keep_active && !bus_acquired) {
        esp_err_t result = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
        if (result != ESP_OK) {
            lcd_note_spi_failure("SPI bus acquire", result);
            if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
            return result;
        }
        bus_acquired = true;
    }

    /* Animation frames live in PSRAM. Passing their address straight to SPI
     * without SPI_TRANS_DMA_USE_PSRAM makes IDF allocate another large internal
     * DMA buffer for every burst. Reuse the existing internal DMA allocation,
     * and keep copy + blocking transmit under the same SPI mutex. Pixel bytes
     * are already in wire order; do not swap them here. */
    if (data != dma_buffer) memcpy(dma_buffer, data, len);
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = dma_buffer,
        .user = (void *)1,
        .flags = keep_active ? SPI_TRANS_CS_KEEP_ACTIVE : 0,
    };
    esp_err_t result = spi_device_transmit(spi_handle, &transaction);
    if (result != ESP_OK) {
        lcd_note_spi_failure("SPI data burst write", result);
    }

    if (!keep_active || result != ESP_OK) lcd_release_bus_locked();
    if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
    return result;
}

static inline void lcd_write_data(const uint8_t *data, size_t len)
{
    if (lcd_suspended) return;
    lcd_write_data_ex(data, len, false);
}

/**
 * @brief Stream a large buffer in DMA-sized chunks, keeping CS asserted
 *        between chunks so the whole frame is sent as one SPI burst.
 *        The last chunk releases CS.
 */
static esp_err_t lcd_write_stream(const uint8_t *data, size_t len)
{
    if (lcd_suspended || spi_handle == NULL || data == NULL || len == 0) return ESP_OK;
    if (!lcd_pixel_retry_ready()) {
        if (spi_mutex != NULL) xSemaphoreTake(spi_mutex, portMAX_DELAY);
        lcd_release_bus_locked();
        if (spi_mutex != NULL) xSemaphoreGive(spi_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    size_t sent = 0;
    while (sent < len) {
        size_t chunk = len - sent;
        if (chunk > DMA_BUF_SIZE) chunk = DMA_BUF_SIZE;
        bool last = (sent + chunk) >= len;
        esp_err_t result = lcd_write_data_ex(data + sent, chunk, !last);
        if (result != ESP_OK) return result; /* Abort this frame, never replay a partial chunk. */
        sent += chunk;
    }
    return ESP_OK;
}

/**
 * @brief Convenience wrapper: send command + optional parameter bytes
 */
static inline esp_err_t write_reg(uint8_t cmd, const uint8_t *data, size_t len)
{
    if (lcd_suspended) return ESP_OK;
    esp_err_t result = lcd_write_cmd_ex(cmd, false);
    if (result != ESP_OK) return result;
    return len > 0 && data != NULL ? lcd_write_data_ex(data, len, false) : ESP_OK;
}

/* ── ST7789 initialization sequence ─────────────────────────────────── */

static void st7789_init_sequence(void)
{
    /* Hardware reset */
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* Software reset */
    lcd_write_cmd(LCD_CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));

    /* Sleep out */
    lcd_write_cmd(LCD_CMD_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Memory Access Control: matches the BOE vendor initialization table. */
    {
        uint8_t madctl = 0x00;
        write_reg(LCD_CMD_MADCTL, &madctl, 1);
    }

    /* Interface Pixel Format: keep 16-bit RGB565 because the complete UI
     * pipeline sends two bytes per pixel. The vendor table's 0x06 setting
     * requires a matching 18-bit/RGB666 pixel stream and cannot be copied
     * independently. */
    {
        uint8_t pixfmt = 0x55;
        write_reg(LCD_CMD_COLMOD, &pixfmt, 1);
    }

    /* Porch control (front/back porch) */
    {
        uint8_t d[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        write_reg(LCD_CMD_PORCTRL, d, 5);
    }

    /* Gate control */
    {
        uint8_t v = 0x35;
        write_reg(LCD_CMD_GCTRL, &v, 1);
    }

    /* VCOM setting */
    {
        uint8_t v = 0x13;
        write_reg(LCD_CMD_VCOMS, &v, 1);
    }

    /* LCM control */
    {
        uint8_t v = 0x2C;
        write_reg(LCD_CMD_LCMCTRL, &v, 1);
    }

    /* VDV and VRH enable */
    {
        uint8_t v = 0x01;
        write_reg(LCD_CMD_VDVVRHEN, &v, 1);
    }

    /* VRH set */
    {
        uint8_t v = 0x0B;
        write_reg(LCD_CMD_VRHS, &v, 1);
    }

    /* VDV set */
    {
        uint8_t v = 0x20;
        write_reg(LCD_CMD_VDVS, &v, 1);
    }

    /* Frame rate control (0x0F ≈ 60 Hz). Increase to reduce tearing risk. */
    {
        uint8_t v = 0x0F;
        write_reg(LCD_CMD_FRCTRL2, &v, 1);
    }

    /* Power control 1 */
    {
        uint8_t d[] = {0xA4, 0xA1};
        write_reg(LCD_CMD_PWCTRL1, d, 2);
    }

    /* Positive gamma: preserve the original screen_mxx-v1.0-ui profile. */
    {
        uint8_t d[] = {0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33,
                       0x50, 0x36, 0x13, 0x14, 0x29, 0x2D};
        write_reg(LCD_CMD_PVGAMCTRL, d, 14);
    }

    /* Negative gamma: preserve the original screen_mxx-v1.0-ui profile. */
    {
        uint8_t d[] = {0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44,
                       0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31};
        write_reg(LCD_CMD_NVGAMCTRL, d, 14);
    }

    /* Inversion on for this panel */
    lcd_write_cmd(LCD_CMD_INVON);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Normal display mode */
    lcd_write_cmd(LCD_CMD_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Keep display output disabled until the UI has filled GRAM with its
     * first complete frame. lcd_ui_init() will issue DISPON afterwards. */

    ESP_LOGI(TAG, "ST7789 initialization sequence complete");
}

/* ── Backlight PWM setup ────────────────────────────────────────────── */

static void lcd_backlight_init(void)
{
    /* Configure LEDC timer for PWM backlight control */
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,   /* 0-255 */
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,               /* 5 kHz PWM */
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* Configure LEDC channel */
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PIN_LCD_BL,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,        /* Start off */
        .hpoint         = 0,
        .intr_type      = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/* ── Public API implementation ──────────────────────────────────────── */

esp_err_t lcd_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD driver (ST7789, %dx%d)", LCD_WIDTH, LCD_HEIGHT);
    lcd_suspended = false;

    /* ── Configure GPIOs ─────────────────────────────────────────── */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_DC) | (1ULL << PIN_LCD_RST) | (1ULL << PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Set initial states */
    gpio_set_level(PIN_LCD_DC, 1);
    gpio_set_level(PIN_LCD_RST, 1);
    gpio_set_level(PIN_LCD_BL, 0);

    /* ── Allocate DMA-capable buffer ─────────────────────────────── */
    dma_buffer = (uint8_t *)heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (dma_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        return ESP_ERR_NO_MEM;
    }

    /* ── Initialize SPI bus ──────────────────────────────────────── */
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,              /* ST7789: write-only (no MISO) */
        .mosi_io_num = PIN_LCD_SDA,
        .sclk_io_num = PIN_LCD_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_MAX_TRANSFER_BYTES,
    };

    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means bus already initialized; that's OK */
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        return ret;
    }

    /* ── Add SPI device ──────────────────────────────────────────── */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_FREQ,
        .mode = 0,                      /* CPOL=0, CPHA=0 */
        .spics_io_num = PIN_LCD_CS,
        .queue_size = 7,
        .pre_cb = spi_pre_transfer_callback,
    };

    ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        return ret;
    }

    /* ── Create SPI mutex for thread-safe access ───────────────── */
    spi_mutex = xSemaphoreCreateMutex();
    if (spi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    operation_mutex = xSemaphoreCreateMutex();
    if (operation_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LCD operation mutex");
        vSemaphoreDelete(spi_mutex);
        spi_mutex = NULL;
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* ── Run ST7789 init sequence ────────────────────────────────── */
    st7789_init_sequence();

    /* ── Initialize backlight PWM ────────────────────────────────── */
    lcd_backlight_init();
    /* Keep the backlight off until lcd_ui_init() has written the first valid
     * full-screen frame. ST7789 GRAM is undefined after power-on and exposing
     * it here causes the brief startup corruption/flash. */
    lcd_set_backlight(0);

    ESP_LOGI(TAG, "LCD driver initialized successfully");
    return ESP_OK;
}

void lcd_deinit(void)
{
    if (spi_handle != NULL) {
        lcd_display_off();
        lcd_set_backlight(0);
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    if (dma_buffer != NULL) {
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
    }
    if (operation_mutex != NULL) {
        vSemaphoreDelete(operation_mutex);
        operation_mutex = NULL;
    }
    if (spi_mutex != NULL) {
        vSemaphoreDelete(spi_mutex);
        spi_mutex = NULL;
    }
    lcd_suspended = false;
}

void lcd_display_on(void)
{
    if (spi_handle == NULL) return;
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    write_reg(LCD_CMD_DISPON, NULL, 0);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_display_off(void)
{
    if (spi_handle == NULL) return;
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    write_reg(LCD_CMD_DISPOFF, NULL, 0);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_prepare_sleep(void)
{
    if (spi_handle == NULL || lcd_suspended) return;

    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    lcd_suspended = true;
    lcd_set_backlight(0);
    lcd_write_cmd_ex(LCD_CMD_DISPOFF, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_write_cmd_ex(LCD_CMD_SLPIN, false);
    vTaskDelay(pdMS_TO_TICKS(120));
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_resume_after_sleep(void)
{
    if (spi_handle == NULL || !lcd_suspended) return;

    /* A short sleep usually preserves the ST7789 register file and GRAM, but
     * that is not reliable after a long light-sleep interval or a panel power/
     * reset transient. Merely sending SLPOUT/NORON/INVON can then leave the
     * controller with an invalid pixel format, address mapping, porch or gamma
     * configuration and the next RGB565 frame appears corrupted.
     *
     * Keep the backlight off and perform the same hardware reset + complete
     * vendor initialization used at cold boot. The sequence intentionally
     * leaves DISPON unset, so the caller can replace undefined GRAM with the
     * first wake frame before exposing the panel. */
    lcd_set_backlight(0);
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    lcd_suspended = false;
    st7789_init_sequence();
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
    ESP_LOGI(TAG, "ST7789 fully reinitialized after light sleep");
}

void lcd_set_backlight(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    uint32_t duty = (percent * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ── Address window & pixel writes ──────────────────────────────────── */

/* ── Framebuffer hook API ───────────────────────────────────────────── */

void lcd_set_framebuffer(uint16_t *buf, int x, int y, int w, int h)
{
    fb_buffer = buf;
    fb_x = x;
    fb_y = y;
    fb_w = w;
    fb_h = h;
}

void lcd_clear_framebuffer(void)
{
    fb_buffer = NULL;
    fb_w = 0;
    fb_h = 0;
}

/* ── Address window & pixel writes ──────────────────────────────────── */

static esp_err_t lcd_set_address_window_impl(int x1, int y1, int x2, int y2)
{
    if (lcd_suspended || spi_handle == NULL) return ESP_ERR_INVALID_STATE;

    /* Apply X/Y offset for the visible window inside 240x320 GRAM */
    int col_start = x1 + LCD_X_GAP;
    int col_end   = x2 + LCD_X_GAP;
    int row_start = y1 + LCD_Y_GAP;
    int row_end   = y2 + LCD_Y_GAP;

    /* Column address set (0x2A) */
    {
        uint8_t d[] = {
            (uint8_t)(col_start >> 8), (uint8_t)(col_start & 0xFF),
            (uint8_t)(col_end   >> 8), (uint8_t)(col_end   & 0xFF),
        };
        esp_err_t result = write_reg(LCD_CMD_CASET, d, 4);
        if (result != ESP_OK) return result;
    }

    /* Row address set (0x2B) */
    {
        uint8_t d[] = {
            (uint8_t)(row_start >> 8), (uint8_t)(row_start & 0xFF),
            (uint8_t)(row_end   >> 8), (uint8_t)(row_end   & 0xFF),
        };
        esp_err_t result = write_reg(LCD_CMD_RASET, d, 4);
        if (result != ESP_OK) return result;
    }

    /* Memory write start (0x2C). Keep CS active so the following pixel data
     * is streamed in the same SPI transaction, reducing tearing. */
    return lcd_write_cmd_ex(LCD_CMD_RAMWR, true);
}


void lcd_set_address_window(int x1, int y1, int x2, int y2)
{
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    (void)lcd_set_address_window_impl(x1, y1, x2, y2);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_write_pixels(const uint8_t *data, size_t len)
{
    if (lcd_suspended || spi_handle == NULL || data == NULL || len == 0) {
        return;
    }
    /* Stream with CS kept active between chunks so a full frame is one burst. */
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    (void)lcd_write_stream(data, len);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_write_frame(int x1, int y1, int x2, int y2,
                     const uint8_t *data, size_t len)
{
    if (lcd_suspended || spi_handle == NULL || data == NULL || len == 0 ||
        !lcd_pixel_retry_ready()) {
        return;
    }

    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    if (lcd_set_address_window_impl(x1, y1, x2, y2) == ESP_OK) {
        (void)lcd_write_stream(data, len);
    }
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

static void lcd_draw_bitmap_locked(int x, int y, int w, int h, const uint16_t *bitmap)
{
    if (spi_handle == NULL || bitmap == NULL || w <= 0 || h <= 0 ||
        !lcd_pixel_retry_ready()) {
        return;
    }

    /* Clip to screen bounds */
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w <= 0 || y + h <= 0) return;

    int src_x = 0, src_y = 0;
    int draw_w = w, draw_h = h;

    if (x < 0) { src_x = -x; draw_w += x; x = 0; }
    if (y < 0) { src_y = -y; draw_h += y; y = 0; }
    if (x + draw_w > LCD_WIDTH)  { draw_w = LCD_WIDTH - x; }
    if (y + draw_h > LCD_HEIGHT) { draw_h = LCD_HEIGHT - y; }
    if (draw_w <= 0 || draw_h <= 0) return;

    /* Full-width bitmap: use DMA burst transfer */
    if (src_x == 0 && draw_w == w) {
        if (lcd_set_address_window_impl(x, y, x + draw_w - 1, y + draw_h - 1) != ESP_OK) return;
        size_t pixels = (size_t)draw_w * draw_h;
        size_t sent = 0;
        while (sent < pixels) {
            size_t chunk = pixels - sent;
            size_t max_pixels = DMA_BUF_SIZE / 2;
            if (chunk > max_pixels) chunk = max_pixels;
            bool last = (sent + chunk) >= pixels;

            uint8_t *buf = dma_buffer;
            for (size_t i = 0; i < chunk; i++) {
                uint16_t color = bitmap[sent + i];
                buf[i * 2 + 0] = (uint8_t)(color >> 8);
                buf[i * 2 + 1] = (uint8_t)(color & 0xFF);
            }
            if (lcd_write_data_ex(dma_buffer, chunk * 2, !last) != ESP_OK) return;
            sent += chunk;
        }
        return;
    }

    /* Partial-width bitmap: row by row */
    for (int row = 0; row < draw_h; row++) {
        int screen_y = y + row;
        if (lcd_set_address_window_impl(x, screen_y, x + draw_w - 1, screen_y) != ESP_OK) return;

        const uint16_t *src = &bitmap[(src_y + row) * w + src_x];
        for (int col = 0; col < draw_w; col++) {
            uint16_t color = src[col];
            uint8_t d[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
            if (lcd_write_data_ex(d, 2, false) != ESP_OK) return;
        }
    }
}

void lcd_draw_bitmap(int x, int y, int w, int h, const uint16_t *bitmap)
{
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    lcd_draw_bitmap_locked(x, y, w, h, bitmap);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

/* ── Drawing primitives ─────────────────────────────────────────────── */

void lcd_fill_screen(lcd_color_t color)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

static void lcd_fill_rect_locked(int x, int y, int w, int h, lcd_color_t color)
{
    if (w <= 0 || h <= 0) return;

    /* Clip to screen bounds */
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_WIDTH)  { w = LCD_WIDTH - x; }
    if (y + h > LCD_HEIGHT) { h = LCD_HEIGHT - y; }
    if (w <= 0 || h <= 0) return;

    /* If framebuffer is active, write into RAM buffer instead of SPI */
    if (fb_buffer != NULL) {
        int x2 = x + w - 1;
        int y2 = y + h - 1;
        int bx1 = fb_x, by1 = fb_y, bx2 = fb_x + fb_w - 1, by2 = fb_y + fb_h - 1;

        int ix1 = (x > bx1) ? x : bx1;
        int iy1 = (y > by1) ? y : by1;
        int ix2 = (x2 < bx2) ? x2 : bx2;
        int iy2 = (y2 < by2) ? y2 : by2;

        if (ix1 <= ix2 && iy1 <= iy2) {
            for (int ry = iy1; ry <= iy2; ry++) {
                uint16_t *row = &fb_buffer[(ry - fb_y) * fb_w];
                for (int rx = ix1; rx <= ix2; rx++) {
                    row[rx - fb_x] = color;
                }
            }
        }
        return;
    }

    if (spi_handle == NULL || !lcd_pixel_retry_ready()) return;

    if (lcd_set_address_window_impl(x, y, x + w - 1, y + h - 1) != ESP_OK) return;

    /* Fill DMA buffer with the color */
    size_t pixel_count = (size_t)w * h;
    size_t buf_pixels = DMA_BUF_SIZE / 2;  /* RGB565 = 2 bytes/pixel */

    /* Pre-fill the DMA buffer */
    for (size_t i = 0; i < buf_pixels; i++) {
        dma_buffer[i * 2 + 0] = (uint8_t)(color >> 8);
        dma_buffer[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    /* Send in chunks, keeping CS active between chunks for tear reduction */
    size_t sent = 0;
    while (sent < pixel_count) {
        size_t chunk = pixel_count - sent;
        if (chunk > buf_pixels) {
            chunk = buf_pixels;
        }
        bool last = (sent + chunk) >= pixel_count;
        if (lcd_write_data_ex(dma_buffer, chunk * 2, !last) != ESP_OK) return;
        sent += chunk;
    }
}

void lcd_fill_rect(int x, int y, int w, int h, lcd_color_t color)
{
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    lcd_fill_rect_locked(x, y, w, h, color);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

static void lcd_draw_pixel_locked(int x, int y, lcd_color_t color)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;

    /* If framebuffer is active, write into RAM buffer */
    if (fb_buffer != NULL) {
        int bx = x - fb_x;
        int by = y - fb_y;
        if (bx >= 0 && bx < fb_w && by >= 0 && by < fb_h) {
            fb_buffer[by * fb_w + bx] = color;
        }
        return;
    }

    if (!lcd_pixel_retry_ready()) return;
    if (lcd_set_address_window_impl(x, y, x, y) != ESP_OK) return;
    uint8_t d[] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    lcd_write_data(d, 2);
}

void lcd_draw_pixel(int x, int y, lcd_color_t color)
{
    if (operation_mutex != NULL) xSemaphoreTake(operation_mutex, portMAX_DELAY);
    lcd_draw_pixel_locked(x, y, color);
    if (operation_mutex != NULL) xSemaphoreGive(operation_mutex);
}

void lcd_draw_hline(int x, int y, int w, lcd_color_t color)
{
    if (y < 0 || y >= LCD_HEIGHT || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > LCD_WIDTH) { w = LCD_WIDTH - x; }
    if (w <= 0) return;
    lcd_fill_rect(x, y, w, 1, color);
}

void lcd_draw_vline(int x, int y, int h, lcd_color_t color)
{
    if (x < 0 || x >= LCD_WIDTH || h <= 0) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > LCD_HEIGHT) { h = LCD_HEIGHT - y; }
    if (h <= 0) return;
    lcd_fill_rect(x, y, 1, h, color);
}

void lcd_draw_rect(int x, int y, int w, int h, lcd_color_t color)
{
    lcd_draw_hline(x, y, w, color);
    lcd_draw_hline(x, y + h - 1, w, color);
    lcd_draw_vline(x, y, h, color);
    lcd_draw_vline(x + w - 1, y, h, color);
}

void lcd_draw_line(int x0, int y0, int x1, int y1, lcd_color_t color)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    dy = -dy;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        lcd_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}
