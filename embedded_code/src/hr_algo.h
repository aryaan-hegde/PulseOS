// Heart rate (BPM) algorithm: C port of analyze() + BpmTracker in
// src/plot_heartrate.py. The Python file is the reference; keep the two in
// step (host_test/compare_hr_algo.py checks they agree).
//
// Pure C with no Pico SDK calls, so it also compiles on a PC for testing.
//
// Usage:
//   hr_algo_reset();
//   for each new green sample:  hr_algo_add_sample(green);
//   every ~0.5 s:
//       float raw = hr_algo_estimate();            // 0 = no reliable reading
//       int shown = bpm_tracker_update(&t, raw, now_seconds);   // 0 = show "--"

#ifndef HR_ALGO_H
#define HR_ALGO_H

#include <stdbool.h>
#include <stdint.h>

#define HR_FS      100              // samples per second (matches heartrate.c)
#define HR_WINDOW  (12 * HR_FS)     // analyse the last 12 s

// ---- Sample buffer + per-window estimate ----------------------------------
void  hr_algo_reset(void);
void  hr_algo_add_sample(uint32_t green);
int   hr_algo_sample_count(void);   // samples currently buffered (max HR_WINDOW)
float hr_algo_estimate(void);       // analyse the buffered window; 0 = none

// Analyse an arbitrary window (oldest sample first). Does not modify x.
// Returns BPM, or 0 when there is no reliable estimate.
float hr_algo_analyze(const float *x, int n);

// ---- Display tracker (smoothing / glitch rejection / hold) ----------------
typedef struct {
    bool   has_value;      // value is valid (locked on)
    double value;          // smoothed BPM
    int    shown;          // integer BPM for the display, 0 = none
    bool   has_last_t;
    double last_t;
    double last_good;
    bool   has_candidate;
    double candidate;
    double candidate_since;
} bpm_tracker_t;

void bpm_tracker_reset(bpm_tracker_t *t);
// raw: latest hr_algo_estimate() (0 = none). now_s: time in seconds.
// Returns the BPM to display, or 0 for "--".
int  bpm_tracker_update(bpm_tracker_t *t, float raw, double now_s);

#endif
