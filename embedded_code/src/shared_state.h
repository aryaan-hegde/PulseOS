// Shared state: the hand-off point between the two cores.
//
//   Core 0 (sensors.c) owns I2C. It reads the RTC, IMU and (later) heart
//   rate sensor, then calls shared_state_publish() with the latest values.
//
//   Core 1 (ui.c) owns the LCD. It calls shared_state_snapshot() to get a
//   consistent copy and draws it. It never touches a sensor.
//
// Both calls copy the whole struct under a lock, so core 1 never sees a
// half-written update (for example a new time with an old date).

#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "imu.h"

typedef struct {
    // ---- Clock (from clock_view) ----
    char time_text[6];          // "14:05" or "--:--"
    char seconds_text[3];       // "09" or "--"
    char date_text[16];         // "Tue 03 Nov 2026", "TIME NOT SET" or "NO CLOCK"
    bool rtc_connected;
    bool rtc_valid;

    // ---- Motion (from imu) ----
    bool imu_connected;
    motion_state_t motion;
    uint16_t steps;             // only counts with IMU_USE_BUILTIN_CLASSIFIER = 1

    // ---- Heart rate (milestone 2+) ----
    uint16_t bpm;               // 0 = no reading to show
} watch_state_t;

// Call once on core 0 before launching core 1.
void shared_state_init(void);

// Core 0: replace the shared copy with *s.
void shared_state_publish(const watch_state_t *s);

// Core 1: copy the latest shared values into *out.
void shared_state_snapshot(watch_state_t *out);

#endif
