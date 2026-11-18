#ifndef HEARTRATE_H
#define HEARTRATE_H

#include <stdbool.h>
#include <stdint.h>

// Uses i2c0 and the pin/speed settings in your existing i2c_bus.h.
bool heartrate_init(void);
// 1 = sample, 0 = no sample yet, -1 = I2C error, -2 = FIFO overflow.
int heartrate_read(uint32_t *red, uint32_t *ir);

// Read every sample waiting in the sensor's 32-sample FIFO (up to max) in one
// I2C transfer. Channel 1 is GREEN with the current slot setup.
// Returns the number of samples read (0 = none yet), -1 = I2C error,
// -2 = FIFO overflowed (samples were lost; caller should reinitialize).
#define HEARTRATE_FIFO_DEPTH 32
int heartrate_read_fifo(uint32_t *green, uint32_t *ir, int max);

#endif
