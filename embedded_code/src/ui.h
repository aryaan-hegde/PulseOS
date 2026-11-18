// Core 1 display loop: owns the LCD (SPI0) and never touches a sensor.
//
// It takes a snapshot of shared_state every UI_FRAME_MS, compares it with
// what is on screen, and redraws and sends only the bands of rows that
// changed. That keeps each update short and stops the flicker of clearing
// and resending the whole screen.

#ifndef UI_H
#define UI_H

#define UI_FRAME_MS 20      // check for changes 50 times a second

// Entry point for core 1: multicore_launch_core1(ui_core1_main).
void ui_core1_main(void);

#endif
