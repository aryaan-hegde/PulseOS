#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "LCD_Helpers/image.h"
#include "LCD_Helpers/GUI_Paint.h"

#define LCD_CS 1
#define LCD_SCL 2
#define LCD_SDA 3
#define LCD_DC 0
#define LCD_RST 12

#define GC9A01A_INREGEN2 0xEF
#define GC9A01A_INREGEN1 0xFE
#define GC9A01A_MADCTL 0x36
#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left
#define GC9A01A_COLMOD 0x3A
#define GC9A01A1_POWER1 0xC1   ///< Power Control 1
#define GC9A01A1_POWER2 0xC3   ///< Power Control 2
#define GC9A01A1_POWER3 0xC4   ///< Power Control 3
#define GC9A01A1_POWER4 0xC9   ///< Power Control 4 
#define GC9A01A_GAMMA1 0xF0    ///< Set gamma 1
#define GC9A01A_GAMMA2 0xF1    ///< Set gamma 2
#define GC9A01A_GAMMA3 0xF2    ///< Set gamma 3
#define GC9A01A_GAMMA4 0xF3    ///< Set gamma 4
#define GC9A01A_FRAMERATE 0xE8 ///< Frame rate control
#define GC9A01A_TEON 0x35      ///< Tearing Effect Line ON
#define GC9A01A_INVON 0x21     ///< Display Inversion ON
#define GC9A01A_SLPOUT 0x11    ///< Sleep Out
#define GC9A01A_DISPON 0x29    ///< Display ON

static uint16_t framebuf[240*240];

void init_spi();
void init_display();
void start_init(const uint8_t *start_seq);

static const uint8_t initcmd[] = {
    GC9A01A_INREGEN2, 0,
    0xEB, 1, 0x14, // ?
    GC9A01A_INREGEN1, 0,
    GC9A01A_INREGEN2, 0,
    0xEB, 1, 0x14, // ?
    0x84, 1, 0x40, // ?
    0x85, 1, 0xFF, // ?
    0x86, 1, 0xFF, // ?
    0x87, 1, 0xFF, // ?
    0x88, 1, 0x0A, // ?
    0x89, 1, 0x21, // ?
    0x8A, 1, 0x00, // ?
    0x8B, 1, 0x80, // ?
    0x8C, 1, 0x01, // ?
    0x8D, 1, 0x01, // ?
    0x8E, 1, 0xFF, // ?
    0x8F, 1, 0xFF, // ?
    0xB6, 2, 0x00, 0x00, // ?
    GC9A01A_MADCTL, 1, MADCTL_MX | MADCTL_BGR,
    GC9A01A_COLMOD, 1, 0x05,
    0x90, 4, 0x08, 0x08, 0x08, 0x08, // ?
    0xBD, 1, 0x06, // ?
    0xBC, 1, 0x00, // ?
    0xFF, 3, 0x60, 0x01, 0x04, // ?
    GC9A01A1_POWER2, 1, 0x13,
    GC9A01A1_POWER3, 1, 0x13,
    GC9A01A1_POWER4, 1, 0x22,
    0xBE, 1, 0x11, // ?
    0xE1, 2, 0x10, 0x0E, // ?
    0xDF, 3, 0x21, 0x0c, 0x02, // ?
    GC9A01A_GAMMA1, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    GC9A01A_GAMMA2, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    GC9A01A_GAMMA3, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    GC9A01A_GAMMA4, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xED, 2, 0x1B, 0x0B, // ?
    0xAE, 1, 0x77, // ?
    0xCD, 1, 0x63, // ?
    // Unsure what this line (from manufacturer's boilerplate code) is
    // meant to do, but users reported issues, seems to work OK without:
    //0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03, // ?
    GC9A01A_FRAMERATE, 1, 0x34,
    0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, // ?
                0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, // ?
                0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07, // ?
    0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00, // ?
    0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98, // ?
    0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00, // ?
    0x98, 2, 0x3e, 0x07, // ?
    GC9A01A_TEON, 0,
    GC9A01A_INVON, 0,
    GC9A01A_SLPOUT, 0x80, // Exit sleep
    GC9A01A_DISPON, 0x80, // Display on
    0x00                  // End of list
};

void init_spi(){
    gpio_set_function(LCD_SDA, GPIO_FUNC_SPI);
    gpio_set_function(LCD_SCL, GPIO_FUNC_SPI);
    spi_init(spi0, 10000000);
    spi_set_format(spi0, 8, 0, 0, SPI_MSB_FIRST);

    gpio_init(LCD_DC);
    gpio_set_dir(LCD_DC, 1);
    gpio_init(LCD_CS);
    gpio_set_dir(LCD_CS, 1);
    gpio_init(LCD_RST);
    gpio_set_dir(LCD_RST, 1);
    
    gpio_put(LCD_CS, 1);
}

void init_display(){
    gpio_put(LCD_RST, 1); sleep_ms(10);
    gpio_put(LCD_RST, 0); sleep_ms(10);
    gpio_put(LCD_RST, 1); sleep_ms(120);
    
    start_init(initcmd);
}

void command_seq(uint8_t cmd, const uint8_t *args, size_t n){
    gpio_put(LCD_CS, 0);
    gpio_put(LCD_DC, 0);                      // command
    spi_write_blocking(spi0, &cmd, 1);
    if (n) {
        gpio_put(LCD_DC, 1);                  // data
        spi_write_blocking(spi0, args, n);
    }
    gpio_put(LCD_CS, 1);
}

void start_init(const uint8_t *start_seq){
    uint8_t cmd;
    while ((cmd = *start_seq++) != 0) {
        uint8_t x = *start_seq++;
        uint8_t n = x & 0x7F;
        command_seq(cmd, start_seq, n);
        start_seq += n;
        if (x & 0x80) sleep_ms(150);
    }
}

void fill_screen(uint16_t color){
    uint8_t col[] = {0, 0, 0, 239};
    uint8_t row[] = {0, 0, 0, 239};
    command_seq(0x2A, col, 4); //column address set
    command_seq(0x2B, row, 4); //row address set

    uint8_t cmd = 0x2C; // memory write
    gpio_put(LCD_CS, 0);
    gpio_put(LCD_DC, 0);
    spi_write_blocking(spi0, &cmd, 1);
    gpio_put(LCD_DC, 1);

    uint8_t px[2] = { color >> 8, color & 0xFF };
    for (int i = 0; i < 240 * 240; i++)
        spi_write_blocking(spi0, px, 2);
    gpio_put(LCD_CS, 1);
}

static void set_window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y){
    uint8_t col[] = {start_x >> 8, start_x & 0xFF, end_x >>8, end_x & 0xFF};
    uint8_t row[] = {start_y >> 8, start_y & 0xFF, end_y >>8, end_y & 0xFF};
    command_seq(0x2A, col, 4);
    command_seq(0x2B, row, 4);
}

void flush_screen(void){
    set_window(0,0,239,239);
    command_seq(0x2C, (const uint8_t *)framebuf, sizeof(framebuf));
}

void display_image(const unsigned char *image, uint16_t x, uint16_t y, uint16_t w, uint16_t h){
    uint8_t cmd = 0x2c; //memory write 
    set_window(x, y, x+w-1, y+h-1);
    command_seq(cmd, image, (size_t)(w*h*2));
}

void turnoff_seq(){
    gpio_put(LCD_CS, 0);
    gpio_put(LCD_DC, 0);
    uint8_t turnoff_cmd = 0x28;
    spi_write_blocking(spi0, &turnoff_cmd, 1);
    gpio_put(LCD_CS, 1);
}

int main(){
    init_spi();
    init_display();
    //fill_screen(0x0000);
    Paint_NewImage(framebuf, 240, 240, 0, 0, 16);
    Paint_Clear(0x0000);
    Paint_DrawString_EN(40, 100, "Words are working", &Font16, 0x0000, 0xFFFF);
    flush_screen();
    //display_image(gImage_1, 90, 90, 60, 60);
    
    //sleep_ms(5000);
    for(;;){}
    return 0;
}