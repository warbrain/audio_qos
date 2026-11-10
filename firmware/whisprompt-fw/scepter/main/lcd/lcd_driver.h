/*
 * LCD Driver for ST7789
 * Resolution: 172(H) x 320(V), RGB565, SPI interface
 * Target: ESP32-S3 with ESP-IDF v6.0.2
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Display dimensions */
#define LCD_WIDTH           172
#define LCD_HEIGHT          320
#define LCD_X_GAP           34   /* X offset in ST7789 internal GRAM (240x320) */
#define LCD_Y_GAP           0

/* Maximum DMA transfer chunk for SPI (64 lines of RGB565).
 * The ESP32 SPI driver/hardware rejects single transactions above this size. */
#define LCD_MAX_TRANSFER_BYTES  (LCD_WIDTH * 24 * 2)   /* 24 lines: keep the internal DMA block small */

/* Color format: RGB565 */
typedef uint16_t lcd_color_t;

/* RGB565 color macros */
#define LCD_COLOR_BLACK     0x0000
#define LCD_COLOR_WHITE     0xFFFF
#define LCD_COLOR_RED       0xF800
#define LCD_COLOR_GREEN     0x07E0
#define LCD_COLOR_BLUE      0x001F
#define LCD_COLOR_YELLOW    0xFFE0
#define LCD_COLOR_CYAN      0x07FF
#define LCD_COLOR_MAGENTA   0xF81F
#define LCD_COLOR_GRAY      0x8410
#define LCD_COLOR_ORANGE    0xFD20

/**
 * @brief Convert RGB888 to RGB565
 */
#define LCD_RGB(r, g, b)    ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3))

/**
 * @brief Initialize the LCD driver (SPI bus, GPIOs, ST7789 controller)
 *
 * Configures SPI2_HOST, sets up DC/CS/RST/BL GPIOs, and sends the full
 * ST7789 initialization sequence including gamma tables. Display output and
 * backlight stay off until the UI writes its first valid frame.
 *
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t lcd_init(void);

/**
 * @brief Deinitialize the LCD driver and free resources
 */
void lcd_deinit(void);

/**
 * @brief Turn display on
 */
void lcd_display_on(void);

/**
 * @brief Turn display off
 */
void lcd_display_off(void);

/**
 * @brief Put the ST7789 panel into sleep mode before ESP light sleep
 */
void lcd_prepare_sleep(void);

/**
 * @brief Fully reset and reinitialize the ST7789 after ESP light sleep while
 *        keeping display output and backlight off. The caller must redraw,
 *        issue display-on, wait for a panel refresh, and enable the backlight.
 */
void lcd_resume_after_sleep(void);

/**
 * @brief Set backlight level (0-100)
 * @param percent 0 = off, 100 = full brightness
 */
void lcd_set_backlight(uint8_t percent);

/**
 * @brief Fill the entire screen with a solid color
 * @param color RGB565 color value
 */
void lcd_fill_screen(lcd_color_t color);

/**
 * @brief Fill a rectangular region with a solid color
 * @param x      Start X coordinate (0 .. LCD_WIDTH-1)
 * @param y      Start Y coordinate (0 .. LCD_HEIGHT-1)
 * @param w      Width in pixels
 * @param h      Height in pixels
 * @param color  RGB565 color value
 */
void lcd_fill_rect(int x, int y, int w, int h, lcd_color_t color);

/**
 * @brief Draw a single pixel
 * @param x      X coordinate
 * @param y      Y coordinate
 * @param color  RGB565 color value
 */
void lcd_draw_pixel(int x, int y, lcd_color_t color);

/**
 * @brief Draw a horizontal line
 */
void lcd_draw_hline(int x, int y, int w, lcd_color_t color);

/**
 * @brief Draw a vertical line
 */
void lcd_draw_vline(int x, int y, int h, lcd_color_t color);

/**
 * @brief Draw a rectangle outline
 */
void lcd_draw_rect(int x, int y, int w, int h, lcd_color_t color);

/**
 * @brief Draw a line (Bresenham)
 */
void lcd_draw_line(int x0, int y0, int x1, int y1, lcd_color_t color);

/**
 * @brief Set the address window for subsequent pixel data writes
 *
 * This is a low-level function; usually you do not need to call it directly.
 *
 * @param x1  Start column (inclusive)
 * @param y1  Start page/row (inclusive)
 * @param x2  End column (inclusive)
 * @param y2  End page/row (inclusive)
 */
void lcd_set_address_window(int x1, int y1, int x2, int y2);

/**
 * @brief Send a raw block of RGB565 pixel data to the current address window
 *
 * The address window must already be set (e.g. by lcd_fill_rect or
 * lcd_set_address_window). Data is sent with DC high.
 *
 * @param data   Pointer to RGB565 pixel buffer
 * @param len    Number of bytes to send
 */
void lcd_write_pixels(const uint8_t *data, size_t len);

/**
 * @brief Atomically set an address window and write its RGB565 pixel data
 *
 * The complete CASET/RASET/RAMWR/data sequence is serialized against panel
 * initialization, display on/off and other frame writes.
 *
 * @param x1    Start column (inclusive)
 * @param y1    Start page/row (inclusive)
 * @param x2    End column (inclusive)
 * @param y2    End page/row (inclusive)
 * @param data  Pointer to RGB565 pixel buffer
 * @param len   Number of bytes to send
 */
void lcd_write_frame(int x1, int y1, int x2, int y2,
                     const uint8_t *data, size_t len);

/**
 * @brief Draw an RGB565 bitmap at (x, y)
 *
 * Bitmap data is an array of uint16_t RGB565 values in row-major order.
 * Pixels with the special value 0xF81F (pure magenta) are treated as
 * transparent and skipped.
 *
 * @param x       Top-left X coordinate
 * @param y       Top-left Y coordinate
 * @param w       Bitmap width in pixels
 * @param h       Bitmap height in pixels
 * @param bitmap  Pointer to uint16_t RGB565 pixel array
 */
void lcd_draw_bitmap(int x, int y, int w, int h, const uint16_t *bitmap);

/**
 * @brief Set an in-memory framebuffer for subsequent draw operations.
 *
 * When a framebuffer is active, lcd_draw_pixel / lcd_fill_rect / etc.
 * write into this buffer instead of sending commands to the LCD.
 * This eliminates flicker by allowing the caller to compose an entire
 * row/region in RAM and flush it with lcd_draw_bitmap() in one DMA burst.
 *
 * @param buf  Pointer to RGB565 pixel buffer (row-major, w*h uint16_t)
 * @param x    Screen X coordinate where this buffer maps to
 * @param y    Screen Y coordinate where this buffer maps to
 * @param w    Buffer width in pixels
 * @param h    Buffer height in pixels
 */
void lcd_set_framebuffer(uint16_t *buf, int x, int y, int w, int h);

/**
 * @brief Clear the active framebuffer hook (resume direct LCD writes)
 */
void lcd_clear_framebuffer(void);

/**
 * @brief Clear screen to black
 */
static inline void lcd_clear(void)
{
    lcd_fill_screen(LCD_COLOR_BLACK);
}

#ifdef __cplusplus
}
#endif
