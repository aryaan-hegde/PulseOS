// BNO085 IMU driver. See imu.h for how to use it.
//
// The BNO085 is not a "read register 0x28" style sensor. It runs its own
// sensor-fusion firmware and talks using CEVA's SH-2 protocol, sent in
// SHTP packets over I2C. CEVA's SH2 library (lib/sh2) handles the protocol.
// This file gives that library four small functions to reach our hardware
// (the "HAL"), turns on the reports we want, and works out the motion state.

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

// If no acceleration sample arrives for this long, assume the sensor is gone.
// At 50 Hz a sample is due every 20 ms, so 500 ms is about 25 missed samples.
#define IMU_DATA_TIMEOUT_MS  500
// How often to try connecting again while the sensor is missing.
#define IMU_RETRY_MS         1000

// Turns a number #define into a string, for building messages.
#define STR_(x) #x
#define TO_STR(x) STR_(x)

// SDA/SCL pins of the bus the IMU is on, only used in error messages.
#define IMU_SDA_PIN (i2c_get_index(IMU_I2C_PORT) ? I2C_BUS1_SDA_PIN : I2C_BUS0_SDA_PIN)
#define IMU_SCL_PIN (i2c_get_index(IMU_I2C_PORT) ? I2C_BUS1_SCL_PIN : I2C_BUS0_SCL_PIN)

// ---- Driver state --------------------------------------------------------
static bool connected = false;       // sh2 session open and reports turned on
static bool need_config = false;     // sensor reset itself, turn reports back on
static bool have_data = false;       // at least one sample since connecting
static bool got_packet = false;      // hal_read() received something
static uint32_t last_sample_ms = 0;  // when the last acceleration sample arrived
static uint32_t last_attempt_ms = 0; // when we last tried to connect
static imu_data_t latest;            // what imu_get_data() hands out

// Ring buffer holding the last IMU_WINDOW_SAMPLES total-acceleration values.
static float window[IMU_WINDOW_SAMPLES];
static int window_pos = 0;           // where the next sample goes
static int window_count = 0;         // how many slots are filled

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ==========================================================================
// HAL: the four functions the SH2 library calls to reach the hardware.
// ==========================================================================

// Called by sh2_open(). Resets the BNO085 so we start from a known state.
static int hal_open(sh2_Hal_t *self) {
    (void)self;
    // A soft-reset command, written as a raw SHTP packet:
    // 4-byte header (length = 5 bytes, channel 1 = "executable", sequence 0)
    // followed by command 1 = reset.
    static const uint8_t reset_packet[5] = { 5, 0, 1, 0, 1 };

    for (int attempt = 0; attempt < 5; attempt++) {
        if (i2c_bus_write(IMU_I2C_PORT, IMU_I2C_ADDR, reset_packet, sizeof reset_packet)) {
            sleep_ms(300);  // the BNO085 needs time to reboot its firmware
            return 0;
        }
        sleep_ms(30);
    }
    return -1;
}

static void hal_close(sh2_Hal_t *self) {
    (void)self;  // nothing to shut down; the I2C bus is shared
}

// Called over and over by sh2_service(). Reads one SHTP packet if the sensor
// has one waiting. Returns its length, or 0 if there is nothing to read.
static int hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    (void)self;

    // Every packet starts with a 4-byte header; the first two bytes are the
    // packet length (the top bit is a "continued" flag, so mask it off).
    // Reading only the header does not use up the packet: the next read
    // starts again from the header.
    uint8_t header[4];
    if (!i2c_bus_read(IMU_I2C_PORT, IMU_I2C_ADDR, header, sizeof header)) {
        return 0;
    }
    unsigned packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_len == 0) return 0;      // nothing waiting
    if (packet_len > len) return 0;     // garbage (e.g. 0xFFFF from an empty bus)

    // Now read the whole packet, header included, in one transfer.
    if (!i2c_bus_read(IMU_I2C_PORT, IMU_I2C_ADDR, buf, packet_len)) {
        return 0;
    }
    *t_us = time_us_32();
    got_packet = true;
    return (int)packet_len;
}

// Called when the library wants to send a command. Returns bytes sent.
static int hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    (void)self;
    return i2c_bus_write(IMU_I2C_PORT, IMU_I2C_ADDR, buf, len) ? (int)len : 0;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}

static sh2_Hal_t hal = { hal_open, hal_close, hal_read, hal_write, hal_get_time_us };

// ==========================================================================
// Motion detection
// ==========================================================================

// Standard deviation of the values in the window: how much the total
// acceleration has been changing. Near 0 when still, bigger with movement.
static float window_std_dev(void) {
    float mean = 0;
    for (int i = 0; i < window_count; i++) mean += window[i];
    mean /= window_count;

    float var = 0;
    for (int i = 0; i < window_count; i++) {
        float d = window[i] - mean;
        var += d * d;
    }
    return sqrtf(var / window_count);
}

static motion_state_t classify(float variation) {
    if (window_count < IMU_WINDOW_SAMPLES / 2) return MOTION_UNKNOWN;  // < 1 s of data
    if (variation < IMU_STILL_MAX) return MOTION_STILL;
    if (variation < IMU_WALK_MAX)  return MOTION_WALKING;
    return MOTION_RUNNING;
}

static void reset_window(void) {
    window_pos = 0;
    window_count = 0;
}

// Handle one new linear acceleration sample.
static void add_sample(float x, float y, float z) {
    latest.x = x;
    latest.y = y;
    latest.z = z;
    latest.total = sqrtf(x * x + y * y + z * z);

    window[window_pos] = latest.total;
    window_pos = (window_pos + 1) % IMU_WINDOW_SAMPLES;
    if (window_count < IMU_WINDOW_SAMPLES) window_count++;

    latest.variation = window_std_dev();
    latest.motion = classify(latest.variation);
    latest.time_ms = now_ms();

    last_sample_ms = latest.time_ms;
    have_data = true;
}

// Convert the BNO085 activity classifier's answer to our motion states.
static motion_state_t from_builtin(uint8_t state) {
    switch (state) {
        case PAC_STILL:   return MOTION_STILL;
        case PAC_ON_FOOT:                        // "on foot" but unsure which
        case PAC_WALKING: return MOTION_WALKING;
        case PAC_RUNNING: return MOTION_RUNNING;
        default:          return MOTION_UNKNOWN; // in vehicle, tilting, ...
    }
}

// ==========================================================================
// SH2 callbacks and setup
// ==========================================================================

// The library calls this for every sensor report that arrives.
static void on_sensor_report(void *cookie, sh2_SensorEvent_t *event) {
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;

    switch (value.sensorId) {
        case SH2_LINEAR_ACCELERATION:
            add_sample(value.un.linearAcceleration.x,
                       value.un.linearAcceleration.y,
                       value.un.linearAcceleration.z);
            break;

        case SH2_STEP_COUNTER:
            latest.steps = value.un.stepCounter.steps;
            break;

        case SH2_PERSONAL_ACTIVITY_CLASSIFIER: {
            uint8_t state = value.un.personalActivityClassifier.mostLikelyState;
            latest.builtin_motion = from_builtin(state);
            if (state < 10) {
                latest.builtin_confidence = value.un.personalActivityClassifier.confidence[state];
            }
            break;
        }
    }
}

// The library calls this for non-sensor events. SH2_RESET means the BNO085
// rebooted (on purpose at startup, or by itself after a glitch) and has
// forgotten which reports were turned on. We can't send commands from inside
// a callback, so just set a flag for imu_update() to handle.
static void on_event(void *cookie, sh2_AsyncEvent_t *event) {
    (void)cookie;
    if (event->eventId == SH2_RESET) need_config = true;
}

// Ask the BNO085 to send one kind of report every interval_us microseconds.
static bool enable_report(sh2_SensorId_t id, uint32_t interval_us, uint32_t sensor_specific) {
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof config);  // all optional features off
    config.reportInterval_us = interval_us;
    config.sensorSpecific = sensor_specific;
    return sh2_setSensorConfig(id, &config) == SH2_OK;
}

static bool configure_reports(void) {
    // Linear acceleration = accelerometer with gravity already removed by
    // the sensor's fusion algorithm. 50 Hz -> one report every 20000 us.
    if (!enable_report(SH2_LINEAR_ACCELERATION, 1000000 / IMU_REPORT_HZ, 0)) return false;

#if IMU_USE_BUILTIN_CLASSIFIER
    // Built-in step counter: reports the running step total.
    if (!enable_report(SH2_STEP_COUNTER, 500000, 0)) return false;
    // Built-in activity classifier. The sensor-specific value is a bit mask
    // of which activities to look for; 0x1FF = all of them.
    if (!enable_report(SH2_PERSONAL_ACTIVITY_CLASSIFIER, 500000, 0x1FF)) return false;
#endif
    return true;
}

// One attempt to connect and turn on the reports.
static bool imu_connect(void) {
    // Quick check first, so a missing sensor costs well under a millisecond
    // instead of the full reset sequence.
    if (!i2c_bus_probe(IMU_I2C_PORT, IMU_I2C_ADDR)) {
        printf("[IMU] BNO085 not found at 0x%02X on I2C%u (SDA=GP%u, SCL=GP%u). "
               "Check wiring and power. Retrying...\n",
               IMU_I2C_ADDR, i2c_get_index(IMU_I2C_PORT), IMU_SDA_PIN, IMU_SCL_PIN);
        return false;
    }

    // Resets the sensor (hal_open) and waits for it to say it is ready.
    if (sh2_open(&hal, on_event, NULL) != SH2_OK) {
        printf("[IMU] BNO085 answered but did not start up. Retrying...\n");
        return false;
    }
    sh2_setSensorCallback(on_sensor_report, NULL);

    if (!configure_reports()) {
        printf("[IMU] BNO085 did not accept the report settings. Retrying...\n");
        sh2_close();
        return false;
    }
    need_config = false;

    printf("[IMU] BNO085 connected at 0x%02X, linear acceleration at %d Hz\n",
           IMU_I2C_ADDR, IMU_REPORT_HZ);
    have_data = false;
    reset_window();
    last_sample_ms = now_ms();  // start the "no data" timer from now
    return true;
}

static void imu_disconnect(const char *reason) {
    printf("[IMU] %s Reconnecting...\n", reason);
    sh2_close();
    // If the sensor vanished mid-transfer it may be holding the bus; free it
    // so the other sensors on the bus keep working.
    i2c_bus_recover(IMU_I2C_PORT);
    connected = false;
    have_data = false;
    reset_window();
    latest.motion = MOTION_UNKNOWN;
}

// ==========================================================================
// Public functions
// ==========================================================================

bool imu_init(void) {
    i2c_bus_init(IMU_I2C_PORT);
    memset(&latest, 0, sizeof latest);
    last_attempt_ms = now_ms();
    connected = imu_connect();
    return connected;
}

void imu_update(void) {
    uint32_t now = now_ms();

    // Not connected: try again once per IMU_RETRY_MS, never block in between.
    if (!connected) {
        if (now - last_attempt_ms >= IMU_RETRY_MS) {
            last_attempt_ms = now;
            connected = imu_connect();
        }
        return;
    }

    // Read waiting packets. Each sh2_service() call reads at most one, so
    // keep going while packets arrive (catches up if the main loop was slow).
    // on_sensor_report() runs from inside these calls.
    for (int i = 0; i < 8; i++) {
        got_packet = false;
        sh2_service();
        if (!got_packet) break;
    }

    // The sensor rebooted by itself: turn the reports back on.
    if (need_config) {
        if (!configure_reports()) {
            imu_disconnect("BNO085 reset itself and could not be set up again.");
            return;
        }
        need_config = false;
        printf("[IMU] BNO085 reset itself; reports turned back on\n");
    }

    // The sensor stopped sending data (unplugged, power glitch, ...).
    if (now_ms() - last_sample_ms > IMU_DATA_TIMEOUT_MS) {
        imu_disconnect("No data from BNO085 for " TO_STR(IMU_DATA_TIMEOUT_MS) " ms.");
    }
}

bool imu_get_data(imu_data_t *out) {
    *out = latest;
    if (!connected || !have_data) {
        out->motion = MOTION_UNKNOWN;
        return false;
    }
    return true;
}

const char *imu_motion_name(motion_state_t m) {
    switch (m) {
        case MOTION_STILL:   return "STILL";
        case MOTION_WALKING: return "WALKING";
        case MOTION_RUNNING: return "RUNNING";
        default:             return "UNKNOWN";
    }
}
