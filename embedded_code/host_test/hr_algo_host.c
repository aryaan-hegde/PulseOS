// PC-side driver for src/hr_algo.c, used by compare_hr_algo.py.
// Reads commands from stdin, one per line:
//   W,v0,v1,...     analyse this window          -> prints the BPM (0 = none)
//   T,raw,now_s     bpm_tracker_update(raw, now) -> prints the shown BPM
//   R               reset the tracker
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/hr_algo.h"

static char line[1 << 20];
static float x[HR_WINDOW * 2];

int main(void) {
    bpm_tracker_t t;
    bpm_tracker_reset(&t);
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'W') {
            int n = 0;
            for (char *p = strchr(line, ','); p && n < HR_WINDOW * 2; p = strchr(p + 1, ','))
                x[n++] = strtof(p + 1, NULL);
            printf("%.4f\n", hr_algo_analyze(x, n));
        } else if (line[0] == 'T') {
            float raw; double now;
            sscanf(line + 2, "%f,%lf", &raw, &now);
            printf("%d\n", bpm_tracker_update(&t, raw, now));
        } else if (line[0] == 'R') {
            bpm_tracker_reset(&t);
            printf("ok\n");
        }
        fflush(stdout);
    }
    return 0;
}
