// BNO085 IMU driver: linear acceleration + simple still/walking/running detection.
//
// How to use it from the main program:
//   imu_init();                 // once at startup
//   while (1) {
//       imu_update();           // every loop pass (at least every 10 ms)
//       imu_data_t d;
//       if (imu_get_data(&d)) { ... use d.x, d.total, d.motion ... }
//   }
//
// imu_update() also handles reconnecting if the sensor is missing or stops
// sending data, so the rest of the program never has to.

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "i2c_bus.h"

// ---- IMU settings (change these to match your wiring) --------------------
// The SDA/SCL pins and bus speed for each port are in i2c_bus.h, because the
// heart rate sensor and RTC share them.
#define IMU_I2C_PORT        i2c1   // which bus the BNO085 is on (i2c0 or i2c1)
#define IMU_I2C_ADDR        0x4A   // 0x4B if the DI jumper is bridged
#define IMU_REPORT_HZ       50     // linear acceleration samples per second

// Set to 1 to also turn on the BNO085's own step counter and activity
// classifier (the sensor works out walking/running itself). Their results
// appear in the steps / builtin_* fields of imu_data_t.
#ifndef IMU_USE_BUILTIN_CLASSIFIER
#define IMU_USE_BUILTIN_CLASSIFIER 0
#endif

// ---- Motion detection tuning ---------------------------------------------
// We look at how much the total acceleration changes (its standard deviation)
// over the last IMU_WINDOW_SAMPLES samples and compare it to these limits.
#define IMU_WINDOW_SAMPLES  100    // 100 samples at 50 Hz = last 2 seconds
#define IMU_STILL_MAX       0.4f   // below this (m/s^2) = still
#define IMU_WALK_MAX        3.0f   // below this = walking, above = running

typedef enum {
    MOTION_UNKNOWN = 0,   // not enough data yet, or sensor disconnected
    MOTION_STILL,
    MOTION_WALKING,
    MOTION_RUNNING,
} motion_state_t;

// Everything the rest of the program needs from the IMU.
typedef struct {
    float x, y, z;          // linear acceleration in m/s^2, gravity removed
    float total;            // size of the acceleration: sqrt(x^2 + y^2 + z^2)
    float variation;        // standard deviation of total over the window
    motion_state_t motion;  // our still/walking/running decision
    uint32_t time_ms;       // when this sample arrived (ms since boot)

    // Only filled in when IMU_USE_BUILTIN_CLASSIFIER is 1:
    uint16_t steps;                   // steps counted by the BNO085
    motion_state_t builtin_motion;    // the BNO085's own activity guess
    uint8_t builtin_confidence;       // how sure it is, 0-100 %
} imu_data_t;

// Start the I2C bus and try to connect to the BNO085.
// Returns true if the sensor answered. If it did not, imu_update() keeps
// retrying in the background, so it is fine to carry on either way.
bool imu_init(void);

// Read any new data from the sensor and update the motion state. Also
// reconnects if the sensor is missing or has stopped sending data.
void imu_update(void);

// Copy the latest values into *out. Returns false (and sets out->motion to
// MOTION_UNKNOWN) if the sensor is not connected or has no data yet.
bool imu_get_data(imu_data_t *out);

// "STILL", "WALKING", ... for printing.
const char *imu_motion_name(motion_state_t m);

#endif
