// Core 1 display loop. See ui.h.
//
// Core 1 also runs the BPM algorithm (hr_algo.c): it is pure number crunching
// (~10 ms every 0.5 s) and core 1 has spare time between screen updates,
// while core 0 must stay free to service the sensors every few ms.
//
// Watch face layout (240 x 240 round screen, rows from the top):
//    22- 34  motion status   "STILL" / "WALKING" / "NO IMU" (small)
//    38- 54  steps           "1234 steps"
//    60-140  heart           beats at the measured BPM, number in the middle
//   144-192  time             "14:05" large + seconds small
//   200-212  date             "Tue 03 Nov 2026"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "LCD_Helpers/GUI_Paint.h"
#include "lcd.h"
#include "ui.h"
#include "shared_state.h"
#include "hr_algo.h"

#define BG          BLACK
#define DARK_RED    0x7800

#define STATUS_Y0   22
#define STATUS_Y1   34
#define STEPS_Y0    38
#define STEPS_Y1    54
#define TIME_Y0     144
#define TIME_Y1     192
#define DATE_Y0     200
#define DATE_Y1     212
#define BPM_Y       76          // Font24 text inside the heart's solid upper part

#define HR_ANALYSE_MS   500     // run the BPM estimate twice a second
#define BEAT_FLASH_MS   150     // heart is bright for this long on each beat

// GUI_Paint's Paint_DrawString_EN() passes its two colours to DrawChar the
// other way round, so (fg, bg) must be given as (bg, fg). Always draw text
// through this helper so the swap lives in one place.
static void draw_text(uint16_t x, uint16_t y, const char *s, sFONT *font,
                      uint16_t fg, uint16_t bg) {
    Paint_DrawString_EN(x, y, s, font, bg, fg);
}

static void clear_rows(uint16_t y0, uint16_t y1) {
    Paint_ClearWindow(0, y0, 240, y1 + 1, BG);
}

static void draw_centered(uint16_t y, const char *s, sFONT *font, uint16_t fg, uint16_t bg) {
    uint16_t w = (uint16_t)(strlen(s) * font->Width);
    uint16_t x = w < 240 ? (240 - w) / 2 : 0;
    draw_text(x, y, s, font, fg, bg);
}

static void draw_time(const watch_state_t *s) {
    // "14:05" in Font48 (140 px) + 4 px gap + "09" in Font24 (34 px), centered.
    const uint16_t x = (240 - (5 * 28 + 4 + 2 * 17)) / 2;
    clear_rows(TIME_Y0, TIME_Y1);
    uint16_t color = s->rtc_valid ? WHITE : GRAY;
    draw_text(x, TIME_Y0, s->time_text, &Font48, color, BG);
    draw_text(x + 5 * 28 + 4, TIME_Y0 + 48 - 24, s->seconds_text, &Font24, GRAY, BG);
    lcd_flush_rows(TIME_Y0, TIME_Y1);
}

static void draw_date(const watch_state_t *s) {
    clear_rows(DATE_Y0, DATE_Y1);
    draw_centered(DATE_Y0, s->date_text, &Font12, GRAY, BG);
    lcd_flush_rows(DATE_Y0, DATE_Y1);
}

static void draw_status(const watch_state_t *s) {
    clear_rows(STATUS_Y0, STATUS_Y1);
    if (!s->imu_connected) {
        draw_centered(STATUS_Y0, "NO IMU", &Font12, DARK_RED, BG);
    } else if (s->motion != MOTION_UNKNOWN) {
        draw_centered(STATUS_Y0, imu_motion_name(s->motion), &Font12, CYAN, BG);
    }
    lcd_flush_rows(STATUS_Y0, STATUS_Y1);
}

static void draw_steps(const watch_state_t *s) {
    char text[20];
    snprintf(text, sizeof text, "%lu steps", (unsigned long)s->steps);
    clear_rows(STEPS_Y0, STEPS_Y1);
    draw_centered(STEPS_Y0, text, &Font16, WHITE, BG);
    lcd_flush_rows(STEPS_Y0, STEPS_Y1);
}

// Heart with the BPM (or "--") in the middle, drawn on the heart's colour.
static void draw_heart(bool bright, int bpm) {
    uint16_t color = bright ? RED : DARK_RED;
    DrawHeart(color);
    char text[8];
    if (bpm > 0) snprintf(text, sizeof text, "%d", bpm);
    else         strcpy(text, "--");
    draw_centered(BPM_Y, text, &Font24, WHITE, color);
    lcd_flush_rows(HEART_Y0, HEART_Y1);
}

void ui_core1_main(void) {
    lcd_init();
    Paint_Clear(BG);
    flush_screen();          // the only full-screen send

    watch_state_t s, shown;
    memset(&shown, 0, sizeof shown);
    bool first = true;

    // Heart rate
    hr_algo_reset();
    bpm_tracker_t tracker;
    bpm_tracker_reset(&tracker);
    hr_result_t result = {0};
    uint32_t last_analyse_ms = 0;
    uint32_t samples[64];

    // Heart animation
    int heart_bpm = -1;          // what the heart currently shows (-1 = not drawn yet)
    bool heart_bright = false;
    uint32_t last_beat_ms = 0;

    while (true) {
        shared_state_snapshot(&s);
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // ---- Feed new samples to the BPM algorithm ----
        int n;
        while ((n = ppg_queue_pop(samples, 64)) > 0) {
            for (int i = 0; i < n; i++) {
                if (samples[i] == PPG_RESET_MARKER) {
                    hr_algo_reset();
                    bpm_tracker_reset(&tracker);
                } else {
                    hr_algo_add_sample(samples[i]);
                }
            }
        }
        if (now_ms - last_analyse_ms >= HR_ANALYSE_MS) {
            last_analyse_ms = now_ms;
            uint32_t t0 = time_us_32();
            float raw = s.hr_connected ? hr_algo_estimate() : 0.0f;
            result.analysis_us = time_us_32() - t0;
            result.bpm_raw = raw;
            result.bpm_shown = (uint16_t)bpm_tracker_update(&tracker, raw, time_us_64() / 1e6);
            if (!s.hr_connected) {
                bpm_tracker_reset(&tracker);
                result.bpm_shown = 0;
            }
            hr_result_publish(&result);
        }

        // ---- Redraw whatever changed ----
        if (first || strcmp(s.seconds_text, shown.seconds_text) != 0
                  || strcmp(s.time_text, shown.time_text) != 0
                  || s.rtc_valid != shown.rtc_valid) {
            draw_time(&s);
        }
        if (first || strcmp(s.date_text, shown.date_text) != 0) {
            draw_date(&s);
        }
        if (first || s.imu_connected != shown.imu_connected || s.motion != shown.motion) {
            draw_status(&s);
        }
        if (first || s.steps != shown.steps) {
            draw_steps(&s);
        }

        // Heart: flash bright once per beat (every 60000/bpm ms), dim otherwise.
        int bpm = result.bpm_shown;
        bool bright = false;
        if (bpm > 0) {
            if (now_ms - last_beat_ms >= 60000u / (uint32_t)bpm) last_beat_ms = now_ms;
            bright = now_ms - last_beat_ms < BEAT_FLASH_MS;
        }
        if (bpm != heart_bpm || bright != heart_bright) {
            heart_bpm = bpm;
            heart_bright = bright;
            draw_heart(bright, bpm);
        }

        shown = s;
        first = false;
        sleep_ms(UI_FRAME_MS);
    }
}
