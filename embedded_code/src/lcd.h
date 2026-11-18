#ifndef LCD_H
#define LCD_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Rows covered by DrawHeart(), for lcd_flush_rows().
#define HEART_Y0 60
#define HEART_Y1 140

void lcd_init(void);
void DrawHeart(uint16_t color);      // draws into the framebuffer only
void HeartWave();
void init_spi();
void init_display();
void start_init(const uint8_t *start_seq);
void command_seq(uint8_t cmd, const uint8_t *args, size_t n);
void fill_screen(uint16_t color);
void flush_screen(void);                        // send the whole framebuffer
void lcd_flush_rows(uint16_t y0, uint16_t y1);  // send rows y0..y1 only
void display_image(const unsigned char *image, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void turnoff_seq();

// Screen sleep. lcd_sleep(): display off + controller sleep (screen goes
// black, the picture in the LCD's memory is kept). lcd_wake(): reverse it
// (blocks ~120 ms while the controller wakes). If LCD_BL_PIN is wired to a
// GPIO, the backlight is switched too, which is where most of the power goes.
void lcd_sleep(void);
void lcd_wake(void);

#endif
