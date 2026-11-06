#ifdef HEARTRATE_TEST
#include <stdio.h>
#include "pico/stdlib.h"
#include "heartrate.h"

/*
 * MAX30101 heart-rate test + live waveform/BPM plot
 *
 * WIRING (SparkFun SEN-16474 breakout):
 *   3.3V -> 3.3V
 *   GND  -> GND
 *   SDA  -> GP36
 *   SCL  -> GP37
 *   INT  -> Not connected
 *
 * Change I2C pins in i2c_bus.h if needed.
 *
 * SETUP:
 *   Firmware files:
 *     src/heartrate.c
 *     src/heartrate_test.c
 *     include/heartrate.h
 *     Existing i2c_bus.c and i2c_bus.h
 *
 *   Add the heartrate_test environment to platformio.ini.
 *   Keep plot_heartrate.py and requirements.txt on your computer.
 *
 * RUN (from the project folder):
 *   1. Build and upload:
 *        pio run -e heartrate_test -t upload
 *
 *   2. Install Python dependencies once:
 *        python -m pip install pyserial numpy scipy matplotlib
 *
 *   3. Find the board's serial port:
 *        pio device list
 *
 *   4. CLOSE PlatformIO Serial Monitor, then start the plot:
 *        python src/plot_heartrate.py --port COM5
 *
 *      Replace COM5 with your actual port.
 *      Adjust the script path if it is outside src/.
 *      If using regular Windows Python, replace "python" with "py".
 *
 * TEST:
 *   Rest a fingertip gently over the sensor and hold still.
 *   The plot shows raw optical readings, a filtered pulse waveform,
 *   and estimated average BPM once enough stable beats are detected.
 *
 * GREEN LED CONFIGURATION:
 *   With green + IR configured in heartrate.c, the first CSV channel
 *   (still named "red" in the code) contains GREEN measurements.
 *   In Python, use analyze(red) to calculate BPM from green.
 *   Using analyze(ir) continues calculating BPM from infrared.
 *
 * TROUBLESHOOTING:
 *   Port busy: close Serial Monitor or other programs using the port.
 *   No plot window: select TkAgg before importing matplotlib.pyplot.
 *   Missing tkinter: use regular Windows Python with Tcl/Tk installed.
 *   No sensor response: check power, ground, I2C pins and pull-ups.
 *   Unstable BPM: hold still and adjust fingertip contact.
 *
 * Firmware changes require rebuilding and uploading.
 * Python changes only require restarting the plot script.
 * BPM is estimated on the computer, not calculated by this firmware.
 */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\n=== PulseOS MAX30101 hardware test ===\n");
    bool connected = false;
    uint32_t last_sample = 0, count = 0;
    while (true)
    {
        if (!connected)
        {
            if (!heartrate_init())
            {
                printf("[HR] Initialization failed; retrying in 2 seconds.\n");
                sleep_ms(2000);
                continue;
            }
            connected = true;
            count = 0;
            last_sample = to_ms_since_boot(get_absolute_time());
        }
        uint32_t red, ir;
        int result = heartrate_read(&red, &ir);
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (result < 0)
        {
            printf("[HR] %s; restarting.\n", result == -2 ? "FIFO overflow" : "I2C error");
            connected = false;
            sleep_ms(100);
        }
        else if (result == 1)
        {
            last_sample = now;
            // Sample index gives uniform 100 Hz timing even when FIFO reads arrive in bursts.
            printf("PPG,%lu,%lu,%lu\n", (unsigned long)count++,
                   (unsigned long)red, (unsigned long)ir);
        }
        else if (now - last_sample > 2000)
        {
            printf("[HR] No samples for 2 seconds; check sensor power. Restarting.\n");
            connected = false;
        }
        else
        {
            sleep_ms(1);
        }
    }
}
#endif
