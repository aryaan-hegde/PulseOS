// Core 0 sensor loop. See sensors.h.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sensors.h"
#include "shared_state.h"
#include "clock_view.h"
#include "rtc.h"
#include "imu.h"
#include "heartrate.h"

#define LINE_MAX_LEN 48

static clock_view_t view;
static watch_state_t state;          // what was last published

static char line[LINE_MAX_LEN];
static int line_len;

// Heart rate sensor.
#define HR_POLL_MS      20      // read the FIFO every 20 ms (~2 samples each time)
#define HR_RETRY_MS     2000    // how often to look for a missing sensor
#define HR_STALL_MS     2000    // no samples for this long = restart the sensor
static bool hr_connected;
static uint32_t hr_last_poll_ms, hr_last_try_ms, hr_last_sample_ms;
static uint32_t ppg_count;      // sample number for the PPG serial stream
static bool ppg_stream;         // "ppg on": print every sample for plot_heartrate.py

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

// Keep the MAX30101 running and hand its samples to core 1.
static void hr_service(uint32_t now_ms) {
    if (!hr_connected) {
        if (now_ms - hr_last_try_ms < HR_RETRY_MS) return;
        hr_last_try_ms = now_ms;
        hr_connected = heartrate_init();
        if (hr_connected) {
            ppg_queue_push(PPG_RESET_MARKER);   // core 1: start a fresh window
            hr_last_sample_ms = hr_last_poll_ms = now_ms;
        }
        return;
    }
    if (now_ms - hr_last_poll_ms < HR_POLL_MS) return;
    hr_last_poll_ms = now_ms;

    uint32_t green[HEARTRATE_FIFO_DEPTH], ir[HEARTRATE_FIFO_DEPTH];
    int n = heartrate_read_fifo(green, ir, HEARTRATE_FIFO_DEPTH);
    if (n < 0) {
        printf("[HR] %s; restarting.\n", n == -2 ? "FIFO overflow" : "I2C error");
        hr_connected = false;
        hr_last_try_ms = now_ms - HR_RETRY_MS + 100;   // retry in 100 ms
        return;
    }
    if (n == 0) {
        if (now_ms - hr_last_sample_ms > HR_STALL_MS) {
            printf("[HR] No samples for 2 seconds; restarting.\n");
            hr_connected = false;
        }
        return;
    }
    hr_last_sample_ms = now_ms;
    for (int i = 0; i < n; i++) {
        ppg_queue_push(green[i]);
        if (ppg_stream) {
            printf("PPG,%lu,%lu,%lu\n", (unsigned long)ppg_count,
                   (unsigned long)green[i], (unsigned long)ir[i]);
        }
        ppg_count++;
    }
}

// Commands handled here; anything else goes to the RTC.
static void handle_command(const char *cmd) {
    if (strcmp(cmd, "ppg on") == 0) {
        ppg_stream = true;
        printf("[core0] PPG stream on\n");
    } else if (strcmp(cmd, "ppg off") == 0) {
        ppg_stream = false;
        printf("[core0] PPG stream off\n");
    } else if (!rtc_command(cmd)) {
        printf("Unknown command \"%s\". Try: set 2026-11-03 14:05:00 | ppg on | ppg off\n", cmd);
    }
}

void sensors_init(void) {
    memset(&state, 0, sizeof state);
    rtc_init();              // keeps retrying inside rtc_update() if it fails
    imu_init();              // keeps retrying inside imu_update() if it fails
    clock_view_init(&view);

    last_pass_us = time_us_32();
    last_status_ms = to_ms_since_boot(get_absolute_time());
    hr_last_try_ms = last_status_ms - HR_RETRY_MS;   // try the HR sensor right away
    printf("Commands:  set YYYY-MM-DD HH:MM:SS  |  time  |  ppg on  |  ppg off\n");
}

void sensors_poll(void) {
    uint32_t now_us = time_us_32();
    uint32_t pass_us = now_us - last_pass_us;
    last_pass_us = now_us;
    if (pass_us > slowest_pass_us) slowest_pass_us = pass_us;

    // ---- Service every sensor ----
    imu_update();            // must run often: reads data, handles reconnects
    rtc_update();            // reconnects if the RTC is missing
    if (read_serial_line()) handle_command(line);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    hr_service(now_ms);

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
    next.hr_connected = hr_connected;

    if (memcmp(&next, &state, sizeof next) != 0) {
        state = next;
        shared_state_publish(&state);
    }

    // ---- Periodic status line (quiet while streaming PPG for the plot) ----
    if (now_ms - last_status_ms >= SENSORS_STATUS_MS && !ppg_stream) {
        last_status_ms = now_ms;
        hr_result_t hr;
        hr_result_get(&hr);
        char hr_text[48];
        if (!hr_connected) {
            snprintf(hr_text, sizeof hr_text, "MISSING");
        } else {
            snprintf(hr_text, sizeof hr_text, "%u bpm (window %.1f, %lu us)",
                     hr.bpm_shown, (double)hr.bpm_raw, (unsigned long)hr.analysis_us);
        }
        printf("[core0] %s:%s  RTC %s  IMU %-7s  HR %s  slowest pass %lu us  dropped %lu\n",
               state.time_text, state.seconds_text,
               state.rtc_connected ? "ok" : "MISSING",
               state.imu_connected ? imu_motion_name(state.motion) : "MISSING",
               hr_text, (unsigned long)slowest_pass_us,
               (unsigned long)ppg_queue_dropped());
        slowest_pass_us = 0;
    }
}
