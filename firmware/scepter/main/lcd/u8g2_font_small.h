/* Auto-generated u8g2 font header — small */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "lcd_driver.h"

#define FONT_SMALL_WIDTH       18
#define FONT_SMALL_HEIGHT      26
#define FONT_SMALL_GLYPH_BYTES 59

int u8g2_font_small_draw_char(int x, int y, char c, lcd_color_t color,
                                   lcd_color_t bg, bool transparent);
int u8g2_font_small_draw_string(int x, int y, const char *str,
                                    lcd_color_t color, lcd_color_t bg, bool transparent);
int u8g2_font_small_string_width(const char *str);
