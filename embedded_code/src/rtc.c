// MCP7940N real-time clock driver. See rtc.h for how to use it.
//
// The MCP7940N is a simple "register" chip: the time lives in registers
// 0x00-0x06, and we read or write them over I2C like a small memory. Each
// value is stored in BCD (binary-coded decimal): every 4 bits hold one
// decimal digit, so 59 seconds is stored as 0x59, not 0x3B. A few spare bits
// in those registers are control/status flags (ST, OSCRUN, VBATEN, ...).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "rtc.h"

// ---- Register addresses (MCP7940N datasheet, Table 5-1) -------------------
#define REG_RTCSEC    0x00  // seconds        bit 7 = ST (start oscillator)
#define REG_RTCMIN    0x01  // minutes
#define REG_RTCHOUR   0x02  // hours          bit 6 = 12/24 hour mode
#define REG_RTCWKDAY  0x03  // weekday 1-7    bit 5 = OSCRUN, 4 = PWRFAIL, 3 = VBATEN
#define REG_RTCDATE   0x04  // day of month
#define REG_RTCMTH    0x05  // month          bit 5 = leap year (read only)
#define REG_RTCYEAR   0x06  // year 00-99
#define REG_CONTROL   0x07  // control        bit 3 = EXTOSC
#define REG_PWRDN     0x18  // 4 bytes: time main power went down
#define REG_PWRUP     0x1C  // 4 bytes: time main power came back
#define REG_SRAM      0x20  // 64 bytes of battery-backed RAM (0x20-0x5F)

// ---- Bits inside those registers ------------------------------------------
#define SEC_ST          0x80  // 1 = run the oscillator (the clock counts)
#define HOUR_12H        0x40  // 1 = 12 hour mode, 0 = 24 hour mode
#define WKDAY_OSCRUN    0x20  // read only: 1 = the crystal is really ticking
#define WKDAY_PWRFAIL   0x10  // 1 = main power was lost (write 0 to clear)
#define WKDAY_VBATEN    0x08  // 1 = switch to the VBAT battery if VCC drops
#define CONTROL_EXTOSC  0x08  // 1 = external clock signal, 0 = crystal

// Two bytes we write into the RTC's battery-backed SRAM after setting the
// time. The SRAM is wiped together with the time when all power is lost, so
// finding these bytes means "the time was set by us and is still good".
static const uint8_t TIME_SET_MARKER[2] = { 0x50, 0x4F };   // "PO"

// ---- Driver state --------------------------------------------------------
static bool connected = false;       // RTC answered and was set up
static bool osc_running = false;     // last value of the OSCRUN bit
static bool time_valid = false;      // time was set and not lost since
static uint32_t last_attempt_ms = 0; // when we last tried to connect

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ==========================================================================
// BCD helpers
// ==========================================================================

// BCD -> normal number. 0x59 -> 5 * 10 + 9 = 59.
static uint8_t bcd_to_dec(uint8_t bcd) {
    return (uint8_t)((bcd >> 4) * 10 + (bcd & 0x0F));
}

// Normal number -> BCD. 59 -> (5 << 4) | 9 = 0x59. Only for 0-99.
static uint8_t dec_to_bcd(uint8_t dec) {
    return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

// ==========================================================================
// Register access
// ==========================================================================

static bool read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_bus_read_reg(RTC_I2C_PORT, RTC_I2C_ADDR, reg, data, len);
}

static bool write_regs(uint8_t reg, const uint8_t *data, size_t len) {
    return i2c_bus_write_reg(RTC_I2C_PORT, RTC_I2C_ADDR, reg, data, len);
}

static bool write_reg(uint8_t reg, uint8_t value) {
    return write_regs(reg, &value, 1);
}

// ==========================================================================
// Calendar helpers
// ==========================================================================

static bool is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static int days_in_month(int month, int year) {
    static const uint8_t days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (month == 2 && is_leap_year(year)) return 29;
    return days[month - 1];
}

// Day of the week for a date, 1 = Monday ... 7 = Sunday.
// (Sakamoto's method: counts days using a small table of month offsets.)
static uint8_t weekday_of(int year, int month, int day) {
    static const int offset[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (month < 3) year -= 1;
    int w = (year + year / 4 - year / 100 + year / 400 + offset[month - 1] + day) % 7;
    return (uint8_t)(w == 0 ? 7 : w);   // the formula gives 0 = Sunday
}

// Check every field is in range (weekday is not checked; we compute it).
static bool time_in_range(const rtc_time_t *t) {
    if (t->year < 2000 || t->year > 2099) return false;
    if (t->month < 1 || t->month > 12) return false;
    if (t->day < 1 || t->day > days_in_month(t->month, t->year)) return false;
    return t->hour < 24 && t->minute < 60 && t->second < 60;
}

// Turn the raw register bytes 0x00-0x06 into normal numbers.
// The masks remove the flag bits that share each register.
static void decode_time(const uint8_t r[7], rtc_time_t *t) {
    t->second  = bcd_to_dec(r[0] & 0x7F);   // drop ST
    t->minute  = bcd_to_dec(r[1] & 0x7F);
    t->hour    = bcd_to_dec(r[2] & 0x3F);   // 24 hour mode: 2 bits of tens
    t->weekday = r[3] & 0x07;               // drop OSCRUN/PWRFAIL/VBATEN
    t->day     = bcd_to_dec(r[4] & 0x3F);
    t->month   = bcd_to_dec(r[5] & 0x1F);   // drop the leap year bit
    t->year    = 2000 + bcd_to_dec(r[6]);
}

// The time the compiler built this file, from the __DATE__ ("Nov  3 2026")
// and __TIME__ ("14:05:09") macros. This is your PC's local time.
static bool build_time(rtc_time_t *t) {
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *date = __DATE__;
    const char *time = __TIME__;

    char mon[4] = { date[0], date[1], date[2], '\0' };
    const char *found = strstr(months, mon);
    if (found == NULL) return false;

    t->month  = (uint8_t)((found - months) / 3 + 1);
    t->day    = (uint8_t)atoi(date + 4);
    t->year   = (uint16_t)atoi(date + 7);
    t->hour   = (uint8_t)atoi(time);
    t->minute = (uint8_t)atoi(time + 3);
    t->second = (uint8_t)atoi(time + 6);
    t->weekday = weekday_of(t->year, t->month, t->day);
    return time_in_range(t);
}

// ==========================================================================
// Oscillator and time setup
// ==========================================================================

static bool read_oscrun(bool *running) {
    uint8_t wkday;
    if (!read_regs(REG_RTCWKDAY, &wkday, 1)) return false;
    *running = (wkday & WKDAY_OSCRUN) != 0;
    return true;
}

// After setting ST, the crystal takes a moment to start swinging. Poll the
// OSCRUN bit (set by the chip itself once it counts real oscillations) for up
// to RTC_OSC_WAIT_MS. This is the check that the clock is *really* running.
static void wait_for_oscillator(void) {
    uint32_t start = now_ms();
    osc_running = false;
    while (now_ms() - start < RTC_OSC_WAIT_MS) {
        if (read_oscrun(&osc_running) && osc_running) {
            printf("[RTC] Oscillator running (OSCRUN = 1) after %lu ms\n",
                   (unsigned long)(now_ms() - start));
            return;
        }
        sleep_ms(10);
    }
    printf("[RTC] WARNING: oscillator did not start (OSCRUN = 0). Check the "
           "32.768 kHz crystal on X1/X2 (pins 1 and 2) and its two load "
           "capacitors to GND. Will keep checking...\n");
}

bool rtc_set_time(const rtc_time_t *t) {
    if (!connected || !time_in_range(t)) return false;
    uint8_t weekday = weekday_of(t->year, t->month, t->day);

    // 1. Stop the clock (ST = 0) so it cannot tick over to the next second
    //    while we are halfway through writing the new time.
    if (!write_reg(REG_RTCSEC, 0x00)) return false;

    // 2. Wait for the chip to report it has stopped (OSCRUN = 0).
    bool running = true;
    for (int i = 0; i < 20 && running; i++) {
        if (!read_oscrun(&running)) return false;
        if (running) sleep_ms(1);
    }

    // 3. Write minutes through year in one go (registers 0x01-0x06).
    //    - hour bit 6 = 0 selects 24 hour mode
    //    - the weekday register also gets VBATEN = 1 (battery backup on) and
    //      PWRFAIL = 0 (clears any old power-fail flag)
    uint8_t regs[6] = {
        dec_to_bcd(t->minute),
        dec_to_bcd(t->hour),                 // 24 hour mode
        (uint8_t)(WKDAY_VBATEN | weekday),
        dec_to_bcd(t->day),
        dec_to_bcd(t->month),
        dec_to_bcd((uint8_t)(t->year - 2000)),
    };
    if (!write_regs(REG_RTCMIN, regs, sizeof regs)) return false;

    // 4. Write the seconds with ST = 1: this starts the clock again.
    if (!write_reg(REG_RTCSEC, (uint8_t)(SEC_ST | dec_to_bcd(t->second)))) return false;

    // 5. Remember (in the battery-backed SRAM) that the time is now valid.
    if (!write_regs(REG_SRAM, TIME_SET_MARKER, sizeof TIME_SET_MARKER)) return false;

    time_valid = true;
    wait_for_oscillator();
    return true;
}

// PWRFAIL = 1 means main power dropped and the backup battery kept the clock
// going. The chip saved when power went down and came back; print that.
static void report_power_failure(void) {
    uint8_t ts[8];
    if (!read_regs(REG_PWRDN, ts, sizeof ts)) return;
    // Each 4-byte stamp: minute, hour, day, month (low 5 bits).
    printf("[RTC] Main power was lost. Down at %02u/%02u %02u:%02u, back at "
           "%02u/%02u %02u:%02u (month/day). The backup battery kept the time.\n",
           bcd_to_dec(ts[3] & 0x1F), bcd_to_dec(ts[2] & 0x3F),
           bcd_to_dec(ts[1] & 0x3F), bcd_to_dec(ts[0] & 0x7F),
           bcd_to_dec(ts[7] & 0x1F), bcd_to_dec(ts[6] & 0x3F),
           bcd_to_dec(ts[5] & 0x3F), bcd_to_dec(ts[4] & 0x7F));
}

// One attempt to find and set up the RTC.
static bool rtc_connect(void) {
    // Quick "anybody home?" check before doing real work.
    if (!i2c_bus_probe(RTC_I2C_PORT, RTC_I2C_ADDR)) {
        printf("[RTC] MCP7940N not found at 0x%02X on I2C%u (SDA=GP%u, SCL=GP%u). "
               "Check wiring, 3.3 V power and pull-up resistors. Retrying...\n",
               RTC_I2C_ADDR, i2c_get_index(RTC_I2C_PORT),
               i2c_get_index(RTC_I2C_PORT) ? I2C_BUS1_SDA_PIN : I2C_BUS0_SDA_PIN,
               i2c_get_index(RTC_I2C_PORT) ? I2C_BUS1_SCL_PIN : I2C_BUS0_SCL_PIN);
        return false;
    }

    // Read all seven time registers and the "time was set" marker.
    uint8_t r[7], marker[2];
    if (!read_regs(REG_RTCSEC, r, sizeof r) ||
        !read_regs(REG_SRAM, marker, sizeof marker)) {
        printf("[RTC] MCP7940N answered but could not be read. Retrying...\n");
        return false;
    }
    connected = true;
    printf("[RTC] MCP7940N found at 0x%02X on I2C%u\n",
           RTC_I2C_ADDR, i2c_get_index(RTC_I2C_PORT));

    // Did main power fail while the battery kept the clock alive?
    if (r[3] & WKDAY_PWRFAIL) report_power_failure();

    // 24 hour mode. Normally already on (we always write it that way), but
    // if something else put the chip in 12 hour mode, convert the hour.
    if (r[2] & HOUR_12H) {
        uint8_t h12 = bcd_to_dec(r[2] & 0x1F);   // 1-12
        bool pm = (r[2] & 0x20) != 0;            // bit 5 = PM in 12 hour mode
        uint8_t h24 = (uint8_t)(h12 % 12 + (pm ? 12 : 0));
        r[2] = dec_to_bcd(h24);
        write_reg(REG_RTCHOUR, r[2]);
        printf("[RTC] Switched from 12 hour to 24 hour mode\n");
    }

    // Battery backup on (VBATEN = 1). Writing the weekday register with
    // PWRFAIL = 0 also clears the power-fail flag. OSCRUN is read only.
    write_reg(REG_RTCWKDAY, (uint8_t)((r[3] & 0x07) | WKDAY_VBATEN));

    // Use the crystal, not an external clock signal (EXTOSC = 0).
    uint8_t control;
    if (read_regs(REG_CONTROL, &control, 1) && (control & CONTROL_EXTOSC)) {
        write_reg(REG_CONTROL, (uint8_t)(control & ~CONTROL_EXTOSC));
    }

    // Is the stored time trustworthy? All three must be true:
    //  - ST = 1: the clock was started (all registers reset to 0, including
    //    ST, if the chip lost all power with no backup battery)
    //  - our marker is in the SRAM (wiped together with the time)
    //  - every field holds a sensible value
    rtc_time_t stored;
    decode_time(r, &stored);
    time_valid = (r[0] & SEC_ST) &&
                 memcmp(marker, TIME_SET_MARKER, sizeof marker) == 0 &&
                 time_in_range(&stored) && stored.weekday >= 1;

    if (time_valid) {
        char buf[32];
        rtc_format(&stored, buf, sizeof buf);
        printf("[RTC] Clock kept its time: %s (not changing it)\n", buf);
        wait_for_oscillator();
        return true;
    }

    printf("[RTC] The clock has no valid time (first use, or it lost all "
           "power with no backup battery).\n");
#if RTC_SET_FROM_BUILD_TIME
    rtc_time_t built;
    if (build_time(&built) && rtc_set_time(&built)) {
        char buf[32];
        rtc_format(&built, buf, sizeof buf);
        printf("[RTC] Set to the time rtc.c was compiled: %s\n", buf);
        printf("[RTC] That may be minutes (or days) old. To set it exactly, type:\n"
               "      set YYYY-MM-DD HH:MM:SS\n");
        return true;
    }
#endif
    // No automatic time: start the oscillator anyway so it counts, and ask
    // for a time. ST = 1 with the seconds already in the register.
    write_reg(REG_RTCSEC, (uint8_t)(r[0] | SEC_ST));
    printf("[RTC] Type this in the serial monitor to set the time:\n"
           "      set YYYY-MM-DD HH:MM:SS\n");
    wait_for_oscillator();
    return true;
}

// The RTC stopped answering: forget it and let rtc_update() look again.
static void rtc_disconnect(void) {
    printf("[RTC] Lost contact with the MCP7940N. Reconnecting...\n");
    // A device unplugged mid-transfer can hold SDA low; free the bus so the
    // other sensors on it keep working.
    i2c_bus_recover(RTC_I2C_PORT);
    connected = false;
    osc_running = false;
    last_attempt_ms = now_ms();
}

// ==========================================================================
// Public functions
// ==========================================================================

bool rtc_init(void) {
    i2c_bus_init(RTC_I2C_PORT);
    last_attempt_ms = now_ms();
    rtc_connect();
    return connected;
}

void rtc_update(void) {
    if (connected) return;
    // Not connected: try again once per RTC_RETRY_MS, never block in between.
    uint32_t now = now_ms();
    if (now - last_attempt_ms >= RTC_RETRY_MS) {
        last_attempt_ms = now;
        rtc_connect();
    }
}

bool rtc_get_time(rtc_time_t *t) {
    if (!connected) return false;

    // Read registers 0x00-0x06 in one burst. If the clock ticks over during
    // the read (e.g. 14:05:59 -> 14:06:00) we could get a mix of old and new
    // values, so read twice and only accept two matching reads.
    uint8_t a[7], b[7];
    if (!read_regs(REG_RTCSEC, a, sizeof a)) { rtc_disconnect(); return false; }
    for (int tries = 0; tries < 3; tries++) {
        if (!read_regs(REG_RTCSEC, b, sizeof b)) { rtc_disconnect(); return false; }
        if (memcmp(a, b, sizeof a) == 0) break;
        memcpy(a, b, sizeof a);
    }

    bool running = (b[3] & WKDAY_OSCRUN) != 0;
    if (running && !osc_running) printf("[RTC] Oscillator is running now (OSCRUN = 1)\n");
    if (!running && osc_running) printf("[RTC] WARNING: oscillator stopped (OSCRUN = 0)\n");
    osc_running = running;

    decode_time(b, t);
    return true;
}

bool rtc_is_connected(void) { return connected; }
bool rtc_is_running(void)   { return connected && osc_running; }
bool rtc_time_is_valid(void) { return connected && time_valid; }

const char *rtc_weekday_name(uint8_t weekday) {
    static const char *names[7] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
    return (weekday >= 1 && weekday <= 7) ? names[weekday - 1] : "???";
}

const char *rtc_month_name(uint8_t month) {
    static const char *names[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                     "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
    return (month >= 1 && month <= 12) ? names[month - 1] : "???";
}

void rtc_format(const rtc_time_t *t, char *buf, size_t size) {
    snprintf(buf, size, "%s %04u-%02u-%02u %02u:%02u:%02u",
             rtc_weekday_name(t->weekday), t->year, t->month, t->day,
             t->hour, t->minute, t->second);
}

bool rtc_command(const char *line) {
    // "time": print the current time once.
    if (strcmp(line, "time") == 0) {
        rtc_time_t t;
        char buf[32];
        if (rtc_get_time(&t)) {
            rtc_format(&t, buf, sizeof buf);
            printf("[RTC] %s%s\n", buf, time_valid ? "" : "  (not set yet)");
        } else {
            printf("[RTC] RTC not connected\n");
        }
        return true;
    }

    // "set YYYY-MM-DD HH:MM:SS" (or HH:MM, seconds = 0).
    if (strncmp(line, "set", 3) == 0) {
        int y, mo, d, h, mi, s = 0;
        int n = sscanf(line + 3, " %d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s);
        if (n < 5) {
            printf("[RTC] Use:  set YYYY-MM-DD HH:MM:SS   e.g.  set 2026-11-03 14:05:00\n");
            return true;
        }
        rtc_time_t t = { 0 };
        // Range-check here so huge numbers can't wrap around in the casts.
        if (y >= 2000 && y <= 2099 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31 &&
            h >= 0 && h <= 23 && mi >= 0 && mi <= 59 && s >= 0 && s <= 59) {
            t.year = (uint16_t)y; t.month = (uint8_t)mo; t.day = (uint8_t)d;
            t.hour = (uint8_t)h;  t.minute = (uint8_t)mi; t.second = (uint8_t)s;
        }
        if (!rtc_set_time(&t)) {
            printf("[RTC] Could not set the time. Check the date is real "
                   "(year 2000-2099) and the RTC is connected.\n");
            return true;
        }
        t.weekday = weekday_of(t.year, t.month, t.day);
        char buf[32];
        rtc_format(&t, buf, sizeof buf);
        printf("[RTC] Time set to %s\n", buf);
        return true;
    }
    return false;
}
