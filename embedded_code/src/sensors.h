// Core 0 sensor loop: owns both I2C buses and the serial monitor.
//
//   sensors_init();             // once, on core 0
//   while (1) sensors_poll();   // as fast as possible, never blocks for long
//
// Each pass services the IMU (which needs attention at least every 10 ms),
// the RTC and serial commands, then publishes anything that changed to
// shared_state for the display core.

#ifndef SENSORS_H
#define SENSORS_H

// Print a status line (RTC/IMU state and the slowest loop pass) this often.
#define SENSORS_STATUS_MS 2000

// Screen sleep: turn the LCD off after this long without movement, and back
// on as soon as the IMU sees movement.
#define SCREEN_SLEEP_MS   30000   // 30 s; raise for real use (e.g. 2 minutes)
#define WAKE_ACCEL_MS2    1.2f    // a jolt above this (m/s^2, gravity removed) wakes it

void sensors_init(void);
void sensors_poll(void);

#endif
