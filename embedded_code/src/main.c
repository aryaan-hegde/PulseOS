#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "LCD_Helpers/image.h"
#include "LCD_Helpers/GUI_Paint.h"

int main(){
    lcd_init();
    Paint_DrawString_EN(40, 180, "Shubh is so cute", &Font16, 0x0000, 0xFFFF);
    //Paint_DrawTime(90, 90, , &Font24, RED, BLACK);
    sleep_ms(5000);
    for(;;){
        DrawHeart(RED);
        sleep_ms(1500);
        DrawHeart(BLUE);
        sleep_ms(500);
    }
    for(;;){}
    return 0;
}