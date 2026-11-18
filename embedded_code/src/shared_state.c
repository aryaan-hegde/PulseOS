// Shared state between core 0 (sensors) and core 1 (display). See shared_state.h.

#include <string.h>
#include "pico/critical_section.h"
#include "hardware/sync.h"
#include "shared_state.h"

static critical_section_t lock;
static watch_state_t shared;

void shared_state_init(void) {
    critical_section_init(&lock);
    memset(&shared, 0, sizeof shared);
}

void shared_state_publish(const watch_state_t *s) {
    critical_section_enter_blocking(&lock);
    shared = *s;
    critical_section_exit(&lock);
}

void shared_state_snapshot(watch_state_t *out) {
    critical_section_enter_blocking(&lock);
    *out = shared;
    critical_section_exit(&lock);
}

// ---- PPG queue ---------------------------------------------------------------
// head is only written by core 0, tail only by core 1. The memory barriers
// make sure the sample is in the buffer before the other core sees the index.
static uint32_t ppg_buf[PPG_QUEUE_SIZE];
static volatile uint32_t ppg_head, ppg_tail, ppg_dropped;

bool ppg_queue_push(uint32_t sample) {
    uint32_t head = ppg_head;
    if (head - ppg_tail >= PPG_QUEUE_SIZE) {
        ppg_dropped++;
        return false;
    }
    ppg_buf[head % PPG_QUEUE_SIZE] = sample;
    __dmb();
    ppg_head = head + 1;
    return true;
}

int ppg_queue_pop(uint32_t *out, int max) {
    uint32_t tail = ppg_tail;
    uint32_t head = ppg_head;
    __dmb();
    int n = 0;
    while (tail != head && n < max) out[n++] = ppg_buf[tail++ % PPG_QUEUE_SIZE];
    __dmb();
    ppg_tail = tail;
    return n;
}

uint32_t ppg_queue_dropped(void) {
    return ppg_dropped;
}

// ---- Heart rate result -------------------------------------------------------
static hr_result_t hr_shared;

void hr_result_publish(const hr_result_t *r) {
    critical_section_enter_blocking(&lock);
    hr_shared = *r;
    critical_section_exit(&lock);
}

void hr_result_get(hr_result_t *out) {
    critical_section_enter_blocking(&lock);
    *out = hr_shared;
    critical_section_exit(&lock);
}
