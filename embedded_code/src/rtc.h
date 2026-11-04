// MCP7940N real-time clock (RTC) driver.
//
// How to use it from the main program:
//   rtc_init();                    // once at startup
//   while (1) {
//       rtc_update();              // every loop pass: reconnects if missing
//       rtc_time_t t;
//       if (rtc_get_time(&t)) { ... use t.hour, t.minute, ... }
//   }
//
// The MCP7940N keeps counting on its own 32.768 kHz crystal, so the time
// survives a reset of the RP2350. It also survives losing main power, as long
// as a backup battery is connected to VBAT (VBATEN is turned on by rtc_init).
//
// Setting the time: rtc_init() only sets the time when the clock has no valid
// time (never set, or it lost all power). It uses the time this file was
// compiled. To set it exactly, type this in the serial monitor:
//   set 2026-11-03 14:05:00

#ifndef RTC_H
#define RTC_H

#include <stdbool.h>
#include <stdint.h>
#include "i2c_bus.h"

// ---- RTC settings (change these to match your wiring) --------------------
// The SDA/SCL pins for each bus are in i2c_bus.h (shared with the other
// sensors). On the PulseOS PCB the RTC is on I2C0: GP4 = SDA, GP5 = SCL.
// To put the RTC on the same wires as the IMU instead, change this to i2c1.
#define RTC_I2C_PORT       i2c0
#define RTC_I2C_ADDR       0x6F    // fixed by the chip, cannot be changed

#define RTC_RETRY_MS       1000    // how often to look for a missing RTC
#define RTC_OSC_WAIT_MS    1000    // how long to wait for the crystal to start

// 1 = when the clock has no valid time, set it to the time rtc.c was compiled.
// 0 = leave it alone and wait for a "set" command from the serial monitor.
#ifndef RTC_SET_FROM_BUILD_TIME
#define RTC_SET_FROM_BUILD_TIME 1
#endif

// One moment in time, in normal (not BCD) numbers.
typedef struct {
    uint8_t  hour;      // 0-23 (24 hour mode)
    uint8_t  minute;    // 0-59
    uint8_t  second;    // 0-59
    uint8_t  day;       // 1-31, day of the month
    uint8_t  month;     // 1-12
    uint16_t year;      // 2000-2099
    uint8_t  weekday;   // 1-7, 1 = Monday ... 7 = Sunday
} rtc_time_t;

// Find the RTC, turn on 24 hour mode and battery backup, start the
// oscillator, and set the time if it has none. Returns true if the RTC
// answered. If it did not, rtc_update() keeps retrying, so it is fine to
// carry on either way.
bool rtc_init(void);

// Call often from the main loop. Retries the connection if the RTC is
// missing and reports when the crystal starts running.
void rtc_update(void);

// Read the current time into *t. Returns false if the RTC is not connected
// or the read failed (then *t is not changed).
bool rtc_get_time(rtc_time_t *t);

// Set the clock. The weekday is worked out from the date, so t->weekday is
// ignored. Returns false if a value is out of range or the RTC is missing.
bool rtc_set_time(const rtc_time_t *t);

bool rtc_is_connected(void);   // RTC answered on the bus
bool rtc_is_running(void);     // OSCRUN = 1: the crystal is ticking
bool rtc_time_is_valid(void);  // the time was set and has not been lost since

// Handle one line typed in the serial monitor. Understands
//   set YYYY-MM-DD HH:MM:SS    (seconds optional)
//   time
// Returns false if the line is not an RTC command.
bool rtc_command(const char *line);

// "Mon".."Sun" and "Jan".."Dec" ("???" if out of range).
const char *rtc_weekday_name(uint8_t weekday);
const char *rtc_month_name(uint8_t month);

// Write the time as "Tue 2026-11-03 14:05:09" into buf (needs 24 bytes).
void rtc_format(const rtc_time_t *t, char *buf, size_t size);

#endif
