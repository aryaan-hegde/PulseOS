// Shared state between core 0 (sensors) and core 1 (display). See shared_state.h.

#include <string.h>
#include "pico/critical_section.h"
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
