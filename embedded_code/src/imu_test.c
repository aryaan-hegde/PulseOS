// Stand-alone IMU test: prints acceleration and motion state to the serial
// monitor. Built only by the "imu_test" environment in platformio.ini, which
// leaves out main.c so the LCD code doesn't run. The normal "proton" build
// skips this whole file.

#ifdef IMU_TEST

#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"

#define PRINT_EVERY_MS 100   // 10 lines per second is easy to read

int main(void) {
    stdio_init_all();
    sleep_ms(1000);          // give the serial monitor a moment to attach
    printf("\n=== PulseOS IMU test ===\n");

    imu_init();              // keeps retrying inside imu_update() if it fails

    uint32_t last_print = 0;
    while (true) {
        imu_update();        // must run often: reads data, handles reconnects

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print < PRINT_EVERY_MS) continue;
        last_print = now;

        imu_data_t d;
        if (!imu_get_data(&d)) continue;   // not connected yet; imu.c prints why

        printf("X:%6.2f  Y:%6.2f  Z:%6.2f  |a|:%6.2f m/s^2  var:%5.2f  %-7s",
               d.x, d.y, d.z, d.total, d.variation, imu_motion_name(d.motion));
#if IMU_USE_BUILTIN_CLASSIFIER
        printf("  | BNO085 says: %-7s %3u%%  steps:%u",
               imu_motion_name(d.builtin_motion), d.builtin_confidence, d.steps);
#endif
        printf("\n");
    }
}

#endif
