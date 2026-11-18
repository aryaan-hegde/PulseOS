// Core 0 sensor loop. See sensors.h.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sensors.h"
#include "shared_state.h"
#include "clock_view.h"
#include "rtc.h"
#include "imu.h"

#define LINE_MAX_LEN 48

static clock_view_t view;
static watch_state_t state;          // what was last published

static char line[LINE_MAX_LEN];
static int line_len;

// Loop timing, so we can prove drawing no longer stalls the sensors.
static uint32_t last_pass_us;
static uint32_t slowest_pass_us;
static uint32_t last_status_ms;

// Collect typed characters into a line without ever waiting for input.
// Returns true when Enter was pressed and line[] holds a full command.
static bool read_serial_line(void) {
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (line_len == 0) continue;      // ignore empty lines / the \n of \r\n
            line[line_len] = '\0';
            line_len = 0;
            printf("\n");
            return true;
        }
        if ((c == '\b' || c == 127) && line_len > 0) {   // backspace
            line_len--;
            printf("\b \b");
            continue;
        }
        if (c >= ' ' && line_len < LINE_MAX_LEN - 1) {
            line[line_len++] = (char)c;
            putchar(c);                       // echo so you can see what you type
        }
    }
    return false;
}

void sensors_init(void) {
    memset(&state, 0, sizeof state);
    rtc_init();              // keeps retrying inside rtc_update() if it fails
    imu_init();              // keeps retrying inside imu_update() if it fails
    clock_view_init(&view);

    last_pass_us = time_us_32();
    last_status_ms = to_ms_since_boot(get_absolute_time());
    printf("Commands:  set YYYY-MM-DD HH:MM:SS   |   time\n");
}

void sensors_poll(void) {
    uint32_t now_us = time_us_32();
    uint32_t pass_us = now_us - last_pass_us;
    last_pass_us = now_us;
    if (pass_us > slowest_pass_us) slowest_pass_us = pass_us;

    // ---- Service every sensor ----
    imu_update();            // must run often: reads data, handles reconnects
    rtc_update();            // reconnects if the RTC is missing
    if (read_serial_line() && !rtc_command(line)) {
        printf("Unknown command \"%s\". Try: set 2026-11-03 14:05:00\n", line);
    }

    // ---- Build the new state and publish it if anything changed ----
    watch_state_t next = state;
    if (clock_view_update(&view)) {
        memcpy(next.time_text, view.time_text, sizeof next.time_text);
        memcpy(next.seconds_text, view.seconds_text, sizeof next.seconds_text);
        memcpy(next.date_text, view.date_text, sizeof next.date_text);
    }
    next.rtc_connected = view.connected;
    next.rtc_valid = view.valid;

    imu_data_t d;
    next.imu_connected = imu_get_data(&d);
    next.motion = d.motion;                       // MOTION_UNKNOWN if not connected
    next.steps = next.imu_connected ? d.steps : state.steps;

    if (memcmp(&next, &state, sizeof next) != 0) {
        state = next;
        shared_state_publish(&state);
    }

    // ---- Periodic status line ----
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_status_ms >= SENSORS_STATUS_MS) {
        last_status_ms = now_ms;
        printf("[core0] %s:%s  RTC %s  IMU %-7s  slowest loop pass %lu us\n",
               state.time_text, state.seconds_text,
               state.rtc_connected ? "ok" : "MISSING",
               state.imu_connected ? imu_motion_name(state.motion) : "MISSING",
               (unsigned long)slowest_pass_us);
        slowest_pass_us = 0;
    }
}
