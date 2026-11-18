// Core 1 display loop. See ui.h.
//
// Watch face layout (240 x 240 round screen, rows from the top):
//    36- 52  motion status   "STILL" / "WALKING" / "NO IMU"
//    60-140  heart            blinks once a second for now (beats in milestone 4)
//   144-192  time             "14:05" large + seconds small
//   200-212  date             "Tue 03 Nov 2026"

#include <string.h>
#include "pico/stdlib.h"
#include "LCD_Helpers/GUI_Paint.h"
#include "lcd.h"
#include "ui.h"
#include "shared_state.h"

#define BG          BLACK
#define DARK_RED    0x7800

#define STATUS_Y0   36
#define STATUS_Y1   52
#define TIME_Y0     144
#define TIME_Y1     192
#define DATE_Y0     200
#define DATE_Y1     212

// GUI_Paint's Paint_DrawString_EN() passes its two colours to DrawChar the
// other way round, so (fg, bg) must be given as (bg, fg). Always draw text
// through this helper so the swap lives in one place.
static void draw_text(uint16_t x, uint16_t y, const char *s, sFONT *font, uint16_t fg) {
    Paint_DrawString_EN(x, y, s, font, BG, fg);
}

static void clear_rows(uint16_t y0, uint16_t y1) {
    Paint_ClearWindow(0, y0, 240, y1 + 1, BG);
}

static void draw_centered(uint16_t y, const char *s, sFONT *font, uint16_t fg) {
    uint16_t w = (uint16_t)(strlen(s) * font->Width);
    uint16_t x = w < 240 ? (240 - w) / 2 : 0;
    draw_text(x, y, s, font, fg);
}

static void draw_time(const watch_state_t *s) {
    // "14:05" in Font48 (140 px) + 4 px gap + "09" in Font24 (34 px), centered.
    const uint16_t x = (240 - (5 * 28 + 4 + 2 * 17)) / 2;
    clear_rows(TIME_Y0, TIME_Y1);
    uint16_t color = s->rtc_valid ? WHITE : GRAY;
    draw_text(x, TIME_Y0, s->time_text, &Font48, color);
    draw_text(x + 5 * 28 + 4, TIME_Y0 + 48 - 24, s->seconds_text, &Font24, GRAY);
    lcd_flush_rows(TIME_Y0, TIME_Y1);
}

static void draw_date(const watch_state_t *s) {
    clear_rows(DATE_Y0, DATE_Y1);
    draw_centered(DATE_Y0, s->date_text, &Font12, GRAY);
    lcd_flush_rows(DATE_Y0, DATE_Y1);
}

static void draw_status(const watch_state_t *s) {
    clear_rows(STATUS_Y0, STATUS_Y1);
    if (!s->imu_connected) {
        draw_centered(STATUS_Y0, "NO IMU", &Font16, DARK_RED);
    } else if (s->motion != MOTION_UNKNOWN) {
        draw_centered(STATUS_Y0, imu_motion_name(s->motion), &Font16, CYAN);
    }
    lcd_flush_rows(STATUS_Y0, STATUS_Y1);
}

static void draw_heart(bool bright) {
    DrawHeart(bright ? RED : DARK_RED);
    lcd_flush_rows(HEART_Y0, HEART_Y1);
}

void ui_core1_main(void) {
    lcd_init();
    Paint_Clear(BG);
    flush_screen();          // the only full-screen send

    watch_state_t s, shown;
    memset(&shown, 0, sizeof shown);
    bool first = true;
    bool heart_bright = false;

    while (true) {
        shared_state_snapshot(&s);

        bool seconds_changed = strcmp(s.seconds_text, shown.seconds_text) != 0;

        if (first || seconds_changed || strcmp(s.time_text, shown.time_text) != 0
                  || s.rtc_valid != shown.rtc_valid) {
            draw_time(&s);
        }
        if (first || strcmp(s.date_text, shown.date_text) != 0) {
            draw_date(&s);
        }
        if (first || s.imu_connected != shown.imu_connected || s.motion != shown.motion) {
            draw_status(&s);
        }
        if (first || seconds_changed) {
            heart_bright = !heart_bright;   // placeholder "alive" blink
            draw_heart(heart_bright);
        }

        shown = s;
        first = false;
        sleep_ms(UI_FRAME_MS);
    }
}
