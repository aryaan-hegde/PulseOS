// Stand-alone RTC test: prints the time to the serial monitor every second
// and lets you set it by typing a command. Built only by the "rtc_test"
// environment in platformio.ini, which leaves out main.c so the LCD code
// doesn't run. The normal "proton" build skips this whole file.
//
// Build/upload with:  pio run -e rtc_test -t upload
// Then open the serial monitor at 115200 baud.
//
// Commands (type in the serial monitor, then press Enter):
//   set 2026-11-03 14:05:00     set the date and time (24 hour)
//   time                        print the time once

#ifdef RTC_TEST

#include <stdio.h>
#include "pico/stdlib.h"
#include "rtc.h"
#include "clock_view.h"

#define LINE_MAX_LEN 48

// Collect typed characters into a line without ever waiting for input.
// Returns true when Enter was pressed and line[] holds a full command.
static bool read_serial_line(char *line, int *len) {
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (*len == 0) continue;          // ignore empty lines / the \n of \r\n
            line[*len] = '\0';
            *len = 0;
            printf("\n");
            return true;
        }
        if ((c == '\b' || c == 127) && *len > 0) {   // backspace
            (*len)--;
            printf("\b \b");
            continue;
        }
        if (c >= ' ' && *len < LINE_MAX_LEN - 1) {
            line[(*len)++] = (char)c;
            putchar(c);                       // echo so you can see what you type
        }
    }
    return false;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);          // give the serial monitor a moment to attach
    printf("\n=== PulseOS RTC test ===\n");
    printf("Commands:  set YYYY-MM-DD HH:MM:SS   |   time\n");

    rtc_init();              // keeps retrying inside rtc_update() if it fails

    clock_view_t view;
    clock_view_init(&view);

    char line[LINE_MAX_LEN];
    int line_len = 0;

    while (true) {
        rtc_update();        // reconnects if the RTC is missing

        if (read_serial_line(line, &line_len) && !rtc_command(line)) {
            printf("Unknown command \"%s\". Try: set 2026-11-03 14:05:00\n", line);
        }

        if (!clock_view_update(&view)) continue;   // nothing new to show

        // ---- This is where the LCD will be updated later ----------------
        // if (view.time_changed) lcd_draw_time(view.time_text, view.time_changed_mask);
        // if (view.date_changed) lcd_draw_date(view.date_text);
        // For now, print what the LCD would show:
        if (view.time_changed || view.seconds_changed) {
            printf("%s:%s  %s%s\n", view.time_text, view.seconds_text, view.date_text,
                   view.connected && !rtc_is_running() ? "  (oscillator NOT running)" : "");
        }
    }
}

#endif
