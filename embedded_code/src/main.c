#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "LCD_Helpers/image.h"
#include "LCD_Helpers/GUI_Paint.h"
#include "clock_view.h"
#include "lcd.h"

#define LINE_MAX_LEN 48

bool heart_is_red = false;

int main(){
    stdio_init_all();
    sleep_ms(1000);          // give the serial monitor a moment to attach
    lcd_init();
    Paint_Clear(0x0000);
    flush_screen();
    rtc_init();              // keeps retrying inside rtc_update() if it fails

    clock_view_t view;
    
    clock_view_init(&view);

    //Paint_DrawString_EN(40, 180, "Shubh is so cute", &Font16, 0x0000, 0xFFFF);

    PAINT_TIME view_formatted;
    while (true) {
        rtc_update();        // reconnects if the RTC is missing
        clock_view_update(&view);
        view_formatted.Day = view.now.day;
        view_formatted.Hour =view.now.hour;
        view_formatted.Min = view.now.minute;
        view_formatted.Month = view.now.month;
        view_formatted.Sec = view.now.second;
        view_formatted.Year = view.now.year;
        Paint_Clear(0x0000);
        Paint_DrawTime(60, 150, &view_formatted, &Font24, RED, BLACK);
        flush_screen();
        heart_is_red = !heart_is_red;
        DrawHeart(heart_is_red ? RED : BLUE);
    }
    return 0;
}