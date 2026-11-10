/*
 * LCD Font Module — 8x16 ASCII Bitmap Font
 * Used for UI text rendering on ST7789 172x320 display
 */

#pragma once

#include <stdint.h>
#include "lcd_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FONT_WIDTH   8
#define FONT_HEIGHT  16

/**
 * @brief Draw a single ASCII character at (x, y) using 8x16 bitmap font
 *
 * @param x      Start X coordinate (top-left)
 * @param y      Start Y coordinate (top-left)
 * @param c      ASCII character (0x20 ~ 0x7E)
 * @param color  Foreground RGB565 color
 * @param bg     Background RGB565 color (set transparent = true to skip bg)
 * @param transparent  If true, only draw foreground pixels
 * @return       Width of the drawn character (always FONT_WIDTH for now)
 */
int lcd_font_draw_char(int x, int y, char c, lcd_color_t color,
                        lcd_color_t bg, bool transparent);

/**
 * @brief Draw a null-terminated string
 *
 * @param x      Start X coordinate
 * @param y      Start Y coordinate
 * @param str    ASCII string
 * @param color  Foreground RGB565 color
 * @param bg     Background RGB565 color
 * @param transparent  If true, skip background pixels
 * @return       Total width drawn in pixels
 */
int lcd_font_draw_string(int x, int y, const char *str,
                          lcd_color_t color, lcd_color_t bg, bool transparent);

/**
 * @brief Get pixel width of a string
 */
int lcd_font_string_width(const char *str);

#ifdef __cplusplus
}
#endif
