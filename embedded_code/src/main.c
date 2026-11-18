// PulseOS main program.
//
//   Core 0: sensors (RTC, IMU, serial commands) -> shared_state   [sensors.c]
//   Core 1: shared_state -> LCD watch face                          [ui.c]
//
// Splitting the work means slow screen updates can never delay the sensors:
// the IMU needs servicing every few ms and the heart rate FIFO overflows
// after ~320 ms, while a full-screen LCD send takes ~90 ms.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "shared_state.h"
#include "sensors.h"
#include "ui.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);          // give the serial monitor a moment to attach
    printf("\n=== PulseOS ===\n");

    shared_state_init();     // before core 1 starts reading it
    multicore_launch_core1(ui_core1_main);

    sensors_init();
    while (true) {
        sensors_poll();
    }
}
