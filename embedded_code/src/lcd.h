#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

void DrawHeart(uint16_t color);
void HeartWave();
void init_spi();
void init_display();
void start_init(const uint8_t *start_seq);
void command_seq(uint8_t cmd, const uint8_t *args, size_t n);
void start_init(const uint8_t *start_seq);
void fill_screen(uint16_t color);
static void set_window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);
void flush_screen(void);
void display_image(const unsigned char *image, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void turnoff_seq();