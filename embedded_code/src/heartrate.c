#include <stdio.h>
#include "pico/stdlib.h"
#include "i2c_bus.h"
#include "heartrate.h"

#define HR_ADDR 0x57
#define HR_BUS i2c0

static bool write8(uint8_t reg, uint8_t value)
{
    return i2c_bus_write_reg(HR_BUS, HR_ADDR, reg, &value, 1);
}

bool heartrate_init(void)
{
    i2c_bus_init(HR_BUS);
    uint8_t id, revision;
    if (!i2c_bus_read_reg(HR_BUS, HR_ADDR, 0xFF, &id, 1) ||
        !i2c_bus_read_reg(HR_BUS, HR_ADDR, 0xFE, &revision, 1))
    {
        printf("[HR] No response at 0x57. Check power, SDA/SCL and pull-ups.\n");
        return false;
    }
    printf("[HR] Part ID: 0x%02X (expected 0x15), revision: 0x%02X\n", id, revision);
    // Related MAX3010x parts share this ID; it does not uniquely identify MAX30101.
    if (id != 0x15)
        return false;

    if (!write8(0x09, 0x40))
        return false; // Software reset
    bool reset_done = false;
    for (unsigned n = 0; n < 100; ++n)
    {
        uint8_t mode;
        if (!i2c_bus_read_reg(HR_BUS, HR_ADDR, 0x09, &mode, 1))
            return false;
        if (!(mode & 0x40))
        {
            reset_done = true;
            break;
        }
        sleep_ms(1);
    }
    if (!reset_done)
        return false;

    // Interrupt pin is unused: poll FIFO instead.
    if (!write8(0x02, 0x00) || !write8(0x03, 0x00) ||
        !write8(0x04, 0x00) || // FIFO write pointer
        !write8(0x05, 0x00) || // Overflow counter
        !write8(0x06, 0x00) || // FIFO read pointer
        !write8(0x08, 0x0F) || // No averaging, no rollover
        !write8(0x0A, 0x27) || // 4096 nA range, 100 samples/s, 411 us / 18 bits
        !write8(0x0C, 0x00) || // Red OFF
        !write8(0x0D, 0x24) || // IR unchanged
        !write8(0x0E, 0x24) || // Green ON (~7.2 mA)
        !write8(0x0F, 0x00) || // Additional green current OFF
        !write8(0x11, 0x23) || // Slot 1 = green, slot 2 = IR
        !write8(0x12, 0x00) || // Slots 3/4 disabled
        !write8(0x09, 0x07))   // Multi-LED mode
        return false;

    printf("[HR] Streaming RED/IR at 100 samples/s. Rest a fingertip on sensor.\n");
    return true;
}

int heartrate_read(uint32_t *red, uint32_t *ir)
{
    if (!red || !ir)
        return -1;
    uint8_t ptr[3]; // Registers 0x04..0x06: write, overflow, read
    if (!i2c_bus_read_reg(HR_BUS, HR_ADDR, 0x04, ptr, 3))
        return -1;
    if (ptr[1] & 0x1F)
        return -2; // Caller reinitializes to discard stale data
    if ((ptr[0] & 0x1F) == (ptr[2] & 0x1F))
        return 0;

    uint8_t data[6];
    if (!i2c_bus_read_reg(HR_BUS, HR_ADDR, 0x07, data, 6))
        return -1;
    *red = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]) & 0x3FFFF;
    *ir = (((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5]) & 0x3FFFF;
    return 1;
}
