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
    uint32_t steps;             // steps today (BNO085 step counter, reset at midnight)

    // ---- Heart rate sensor ----
    bool hr_connected;          // MAX30101 answered and is streaming
} watch_state_t;

// Call once on core 0 before launching core 1.
void shared_state_init(void);

// Core 0: replace the shared copy with *s.
void shared_state_publish(const watch_state_t *s);

// Core 1: copy the latest shared values into *out.
void shared_state_snapshot(watch_state_t *out);

// ---- PPG sample queue: core 0 -> core 1 -----------------------------------
// Core 0 pushes every green sample (100 per second); core 1 pops them into
// the BPM algorithm. Single producer / single consumer, no lock needed.
// 512 samples = 5 s of slack if core 1 is busy drawing.
#define PPG_QUEUE_SIZE    512
#define PPG_RESET_MARKER  0xFFFFFFFFu   // pushed after a sensor (re)start: clear history

bool     ppg_queue_push(uint32_t sample);           // core 0; false if full (sample dropped)
int      ppg_queue_pop(uint32_t *out, int max);     // core 1; returns how many were copied
uint32_t ppg_queue_dropped(void);                   // samples lost because the queue was full

// ---- Heart rate result: core 1 -> core 0 (for the serial status line) ------
typedef struct {
    uint16_t bpm_shown;         // what the watch face shows, 0 = "--"
    float    bpm_raw;           // latest 12 s window estimate, 0 = none
    uint32_t analysis_us;       // how long that estimate took to compute
} hr_result_t;

void hr_result_publish(const hr_result_t *r);
void hr_result_get(hr_result_t *out);

#endif
