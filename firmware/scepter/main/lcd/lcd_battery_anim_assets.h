#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_BATTERY_ANIM_FRAME_COUNT 5
#define LCD_BATTERY_ANIM_NUMBER_FRAME 2
#define LCD_BATTERY_ANIM_FPS 7
#define LCD_BATTERY_ANIM_NUMBER_HOLD_MS 2500
#define LCD_BATTERY_TRANSPARENT_COLOR 0xF81F

typedef struct {
    uint16_t color;
    uint16_t count;
} lcd_battery_rle_run_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t run_count;
    const lcd_battery_rle_run_t *runs;
} lcd_battery_rle_image_t;

extern const lcd_battery_rle_image_t lcd_battery_anim_frames[LCD_BATTERY_ANIM_FRAME_COUNT];
extern const lcd_battery_rle_image_t lcd_battery_face_digits[10];
extern const lcd_battery_rle_image_t lcd_battery_face_digits_low[10];
extern const lcd_battery_rle_image_t lcd_battery_face_percent;
extern const lcd_battery_rle_image_t lcd_battery_face_percent_low;
extern const lcd_battery_rle_image_t lcd_battery_status_digits[10];
extern const lcd_battery_rle_image_t lcd_battery_status_digits_low[10];
extern const lcd_battery_rle_image_t lcd_battery_status_percent;
extern const lcd_battery_rle_image_t lcd_battery_status_percent_low;
extern const lcd_battery_rle_image_t lcd_battery_status_10;
extern const lcd_battery_rle_image_t lcd_battery_status_20;
extern const lcd_battery_rle_image_t lcd_battery_status_full;
extern const lcd_battery_rle_image_t lcd_battery_status_charge;
extern const lcd_battery_rle_image_t lcd_battery_status_charge_icon_only;

#ifdef __cplusplus
}
#endif
