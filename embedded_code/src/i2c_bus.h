// Shared I2C bus setup for every sensor on the watch.
//
// All I2C pins and speeds live here, so each sensor driver (IMU, heart rate,
// RTC) just says "I am on bus X at address Y" and never touches pins itself.
//
// Sensors and addresses:
//   BNO085   IMU          0x4A (0x4B if the DI jumper is bridged)
//   MAX30101 heart rate   0x57
//   MCP7940N clock        0x6F
//
// On the PulseOS PCB the BNO085 has its own bus (I2C1), and the MAX30101 and
// MCP7940N share I2C0. When prototyping with breakout boards you can put all
// three on one bus: wire them in parallel and point each driver at the same
// port.

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"

// ---- Pin and speed settings (change these to match your wiring) ----------
// RP2350 rule: GPIO n can only be I2C(n/2 % 2), SDA when n is even, SCL when
// n is odd. So GP36/GP37 are I2C0 and GP6/GP7 are I2C1.

// I2C0: MAX30101 heart rate + MCP7940N RTC on the PCB
#define I2C_BUS0_SDA_PIN 36
#define I2C_BUS0_SCL_PIN 37
#define I2C_BUS0_BAUD_HZ 400000 // 400 kHz "fast mode", all parts support it

// I2C1: BNO085 IMU on the PCB
#define I2C_BUS1_SDA_PIN 6
#define I2C_BUS1_SCL_PIN 7
#define I2C_BUS1_BAUD_HZ 400000

// Set up the pins and peripheral for one bus (i2c0 or i2c1). It is safe to
// call this from every driver: only the first call does anything.
void i2c_bus_init(i2c_inst_t *i2c);

// Write len bytes to a device. Returns true if the device ACKed everything.
bool i2c_bus_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data, size_t len);

// Read len bytes from a device. Returns true if all bytes were read.
bool i2c_bus_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data, size_t len);

// Register access for "normal" register-based chips (RTC, heart rate sensor).
// read_reg:  read len bytes starting at register reg.
// write_reg: write len bytes starting at register reg (len <= 16).
#define I2C_BUS_MAX_REG_WRITE 16
bool i2c_bus_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
bool i2c_bus_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);

// Quick "is anybody at this address?" check (a 1-byte read).
bool i2c_bus_probe(i2c_inst_t *i2c, uint8_t addr);

// Unstick a bus where a device is holding SDA low (for example after a
// sensor was unplugged in the middle of a transfer), then set it up again.
void i2c_bus_recover(i2c_inst_t *i2c);

#endif
