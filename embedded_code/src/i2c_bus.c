// Shared I2C bus setup. See i2c_bus.h for the pin settings.

#include <stdio.h>
#include "pico/stdlib.h"
#include "i2c_bus.h"

// Settings and "already set up?" flag for each of the two I2C peripherals.
typedef struct {
    uint sda_pin;
    uint scl_pin;
    uint baud_hz;
    bool ready;
} i2c_bus_t;

static i2c_bus_t buses[2] = {
    { I2C_BUS0_SDA_PIN, I2C_BUS0_SCL_PIN, I2C_BUS0_BAUD_HZ, false },
    { I2C_BUS1_SDA_PIN, I2C_BUS1_SCL_PIN, I2C_BUS1_BAUD_HZ, false },
};

static i2c_bus_t *get_bus(i2c_inst_t *i2c) {
    return &buses[i2c_get_index(i2c)];
}

// Every transfer gets a time limit so a stuck or missing device can never
// freeze the program. At 400 kHz one byte takes about 25 us; the extra margin
// covers the BNO085 holding SCL low ("clock stretching") while it gets ready.
static uint32_t timeout_us(size_t len) {
    return 2000 + 100 * (uint32_t)len;
}

void i2c_bus_init(i2c_inst_t *i2c) {
    i2c_bus_t *bus = get_bus(i2c);
    if (bus->ready) return;

    // Catch pin typos early: the pins must belong to this I2C peripheral.
    uint index = i2c_get_index(i2c);
    if ((bus->sda_pin / 2) % 2 != index || bus->sda_pin % 2 != 0 ||
        (bus->scl_pin / 2) % 2 != index || bus->scl_pin % 2 != 1) {
        printf("[I2C] WARNING: GP%u/GP%u are not the SDA/SCL pins of I2C%u\n",
               bus->sda_pin, bus->scl_pin, index);
    }

    i2c_init(i2c, bus->baud_hz);
    gpio_set_function(bus->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(bus->scl_pin, GPIO_FUNC_I2C);
    // The breakout boards already have pull-up resistors; the RP2350's weak
    // internal ones are just a backup in case a board has none.
    gpio_pull_up(bus->sda_pin);
    gpio_pull_up(bus->scl_pin);

    bus->ready = true;
}

bool i2c_bus_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data, size_t len) {
    int n = i2c_write_timeout_us(i2c, addr, data, len, false, timeout_us(len));
    return n == (int)len;
}

bool i2c_bus_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data, size_t len) {
    int n = i2c_read_timeout_us(i2c, addr, data, len, false, timeout_us(len));
    return n == (int)len;
}

bool i2c_bus_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    // Step 1: send the register number. "nostop = true" leaves the bus open,
    // so step 2 starts with a repeated START instead of STOP + START.
    if (i2c_write_timeout_us(i2c, addr, &reg, 1, true, timeout_us(1)) != 1) {
        return false;
    }
    // Step 2: read; the device sends reg, reg+1, reg+2, ...
    return i2c_bus_read(i2c, addr, data, len);
}

bool i2c_bus_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    // One transfer: the register number first, then the data bytes.
    uint8_t buf[1 + I2C_BUS_MAX_REG_WRITE];
    if (len > I2C_BUS_MAX_REG_WRITE) return false;
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) buf[1 + i] = data[i];
    return i2c_bus_write(i2c, addr, buf, len + 1);
}

bool i2c_bus_probe(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t dummy;
    return i2c_bus_read(i2c, addr, &dummy, 1);
}

void i2c_bus_recover(i2c_inst_t *i2c) {
    i2c_bus_t *bus = get_bus(i2c);

    // Take the pins back from the I2C hardware so we can wiggle them by hand.
    // A pin is "released" (pulled high by the resistor) when it is an input,
    // and driven low when it is an output set to 0, just like real I2C.
    i2c_deinit(i2c);
    gpio_init(bus->sda_pin);
    gpio_init(bus->scl_pin);
    gpio_pull_up(bus->sda_pin);
    gpio_pull_up(bus->scl_pin);
    gpio_put(bus->sda_pin, 0);
    gpio_put(bus->scl_pin, 0);

    // A device stuck in the middle of sending a byte lets go of SDA after we
    // give it enough clock pulses (at most 9) to finish that byte.
    for (int i = 0; i < 9 && !gpio_get(bus->sda_pin); i++) {
        gpio_set_dir(bus->scl_pin, GPIO_OUT); sleep_us(5);   // SCL low
        gpio_set_dir(bus->scl_pin, GPIO_IN);  sleep_us(5);   // SCL high
    }

    // Send a STOP condition (SDA goes low -> high while SCL is high) so every
    // device on the bus goes back to idle.
    gpio_set_dir(bus->sda_pin, GPIO_OUT); sleep_us(5);
    gpio_set_dir(bus->sda_pin, GPIO_IN);  sleep_us(5);

    // Hand the pins back to the I2C hardware.
    bus->ready = false;
    i2c_bus_init(i2c);
}
