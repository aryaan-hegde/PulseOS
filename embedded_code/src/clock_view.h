// Clock view: the bridge between the RTC and the LCD.
//
// It reads the RTC a few times a second and turns the time into ready-to-draw
// text, plus flags saying which parts changed since the last update. The
// LCD code never talks to the RTC itself; it only draws these strings.
//
// How the LCD code will use it (once the display driver exists):
//   clock_view_t view;
//   clock_view_init(&view);
//   while (1) {
//       rtc_update();
//       if (clock_view_update(&view)) {           // true = something changed
//           if (view.time_changed)  lcd_draw_time(view.time_text, view.time_changed_mask);
//           if (view.date_changed)  lcd_draw_date(view.date_text);
//       }
//   }
// Only redrawing the changed parts (and, inside "14:05", only the changed
// characters) is what keeps the screen from flickering.

#ifndef CLOCK_VIEW_H
#define CLOCK_VIEW_H

#include <stdbool.h>
#include <stdint.h>
#include "rtc.h"

// How often to read the RTC. Reading 4 times a second (instead of exactly
// once) means the screen changes within 1/4 s of the real tick and never
// skips a second because the RP2350's timer and the RTC drift apart.
// Each read is only ~0.5 ms of I2C traffic.
#define CLOCK_VIEW_POLL_MS  250

typedef struct {
    // ---- What to show ----
    char time_text[6];      // "14:05", or "--:--" if there is no RTC
    char seconds_text[3];   // "09",    or "--"
    char date_text[16];     // "Tue 03 Nov 2026", "TIME NOT SET" or "NO CLOCK"

    // ---- What changed in the last clock_view_update() ----
    bool time_changed;          // time_text is different
    uint8_t time_changed_mask;  // bit i set = time_text[i] changed (bit 2 is ':')
    bool seconds_changed;       // seconds_text is different
    bool date_changed;          // date_text is different

    // ---- The raw values, if you need numbers instead of text ----
    rtc_time_t now;             // last time read (only meaningful if connected)
    bool connected;             // RTC answered
    bool valid;                 // time has been set and not lost
    uint32_t last_poll_ms;      // when the RTC was last read
} clock_view_t;

// Start with everything blank, so the first update marks every part changed
// and the LCD draws the whole face once.
void clock_view_init(clock_view_t *v);

// Read the RTC (at most every CLOCK_VIEW_POLL_MS) and refresh the text and
// the changed flags. Returns true if any text changed. Never blocks: if the
// RTC is missing it shows "--:--" / "NO CLOCK".
bool clock_view_update(clock_view_t *v);

#endif
