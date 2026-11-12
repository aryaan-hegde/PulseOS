#ifndef HEARTRATE_H
#define HEARTRATE_H

#include <stdbool.h>
#include <stdint.h>

// Uses i2c0 and the pin/speed settings in your existing i2c_bus.h.
bool heartrate_init(void);
// 1 = sample, 0 = no sample yet, -1 = I2C error, -2 = FIFO overflow.
int heartrate_read(uint32_t *red, uint32_t *ir);

#endif
