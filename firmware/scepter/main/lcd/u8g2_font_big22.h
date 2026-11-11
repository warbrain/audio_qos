/* Auto-generated u8g2 font header — big22 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "lcd_driver.h"

#define FONT_BIG22_WIDTH       20
#define FONT_BIG22_HEIGHT      30
#define FONT_BIG22_GLYPH_BYTES 75

int u8g2_font_big22_draw_char(int x, int y, char c, lcd_color_t color,
                                   lcd_color_t bg, bool transparent);
int u8g2_font_big22_draw_string(int x, int y, const char *str,
                                    lcd_color_t color, lcd_color_t bg, bool transparent);
int u8g2_font_big22_string_width(const char *str);
