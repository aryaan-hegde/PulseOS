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

void sensors_init(void);
void sensors_poll(void);

#endif
