// Clock view: turns RTC readings into text for the LCD. See clock_view.h.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "clock_view.h"

void clock_view_init(clock_view_t *v) {
    memset(v, 0, sizeof *v);   // empty strings: the first update changes all of them
    // Pretend the last read was long ago so the first update reads right away.
    v->last_poll_ms = to_ms_since_boot(get_absolute_time()) - CLOCK_VIEW_POLL_MS;
}

bool clock_view_update(clock_view_t *v) {
    v->time_changed = false;
    v->time_changed_mask = 0;
    v->seconds_changed = false;
    v->date_changed = false;

    // Only read the RTC every CLOCK_VIEW_POLL_MS.
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - v->last_poll_ms < CLOCK_VIEW_POLL_MS) return false;
    v->last_poll_ms = now;

    // Build the new text into temporary buffers.
    char time_text[sizeof v->time_text];
    char seconds_text[sizeof v->seconds_text];
    char date_text[sizeof v->date_text];

    v->connected = rtc_get_time(&v->now);
    v->valid = v->connected && rtc_time_is_valid();

    if (!v->connected) {
        // No RTC on the bus (rtc_update() keeps looking for it).
        strcpy(time_text, "--:--");
        strcpy(seconds_text, "--");
        strcpy(date_text, "NO CLOCK");
    } else {
        // "% 100" only tells the compiler each number fits in 2 digits.
        snprintf(time_text, sizeof time_text, "%02u:%02u",
                 v->now.hour % 100u, v->now.minute % 100u);
        snprintf(seconds_text, sizeof seconds_text, "%02u", v->now.second % 100u);
        if (v->valid) {
            // e.g. "Tue 03 Nov 2026" (15 characters)
            snprintf(date_text, sizeof date_text, "%s %02u %s %04u",
                     rtc_weekday_name(v->now.weekday), v->now.day,
                     rtc_month_name(v->now.month), v->now.year);
        } else {
            strcpy(date_text, "TIME NOT SET");
        }
    }

    // Compare with what is on the screen now, character by character for the
    // time so the LCD can redraw single digits.
    for (int i = 0; i < 5; i++) {
        if (time_text[i] != v->time_text[i]) v->time_changed_mask |= (uint8_t)(1u << i);
    }
    v->time_changed = v->time_changed_mask != 0;
    v->seconds_changed = strcmp(seconds_text, v->seconds_text) != 0;
    v->date_changed = strcmp(date_text, v->date_text) != 0;

    // Remember the new text for next time.
    strcpy(v->time_text, time_text);
    strcpy(v->seconds_text, seconds_text);
    strcpy(v->date_text, date_text);

    return v->time_changed || v->seconds_changed || v->date_changed;
}
