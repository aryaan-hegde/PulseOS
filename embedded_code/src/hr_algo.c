// Heart rate algorithm. See hr_algo.h. Each step mirrors a function in
// src/plot_heartrate.py; the Python names are given in the comments.
//
// All working arrays are static (core 1 only has a 2 KB stack).

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "hr_algo.h"

// ---- Tuning: identical to the constants in plot_heartrate.py ---------------
#define MIN_IBI_SAMPLES     30          // 60/200 BPM * FS
#define MAX_IBI_NUM         6000        // interval <= 60/35 s  <=>  35*d <= 6000
#define MAX_IBI_DEN         35
#define PROMINENCE_FRAC     0.30f
#define OUTLIER_FRAC        0.25f
#define MIN_GOOD_INTERVALS  3
#define MIN_GOOD_RATIO      0.6f
#define MIN_AMPLITUDE       8.0f
#define SATURATION          262000u
#define MAX_SATURATED_FRAC  0.05f
#define MIN_AUTOCORR        0.5f
#define AGREE_FRAC          0.10f
#define MOTION_K            6.0f
#define MOTION_PAD          25          // 0.25 s
#define MAX_MOTION_FRAC     0.4f
#define MEDIAN_SIZE         151         // int(1.5 * FS) | 1
#define EDGE_START          (HR_FS / 2) // ignore first 0.5 s (filter edge)
#define EDGE_END            (HR_FS / 4) // ignore newest 0.25 s
#define AC_LAG_LO           30          // int(MIN_IBI * FS)
#define AC_LAG_HI           171         // int(MAX_IBI * FS)

// Band-pass 0.7-3.5 Hz, Butterworth order 2, fs = 100 Hz, as second-order
// sections [b0 b1 b2 1 a1 a2]. From scipy.signal.butter(..., output='sos').
static const float SOS[2][5] = {
    {0.00686786572868267f, 0.01373573145736534f, 0.00686786572868267f,
     -1.7860235008089684f, 0.8203639438488808f},
    {1.0f, -2.0f, 1.0f, -1.9480658464669813f, 0.9504799240320247f},
};
// scipy.signal.sosfilt_zi(SOS): filter state for a constant input of 1.
static const float ZI[2][2] = {
    {0.7931061731269553f, -0.6494019917636463f},
    {-0.7999740388556157f, 0.7999740388556168f},
};
#define PADLEN 15   // sosfiltfilt default pad length for this filter

// ---- Sample ring buffer ----------------------------------------------------
static uint32_t ring[HR_WINDOW];
static int ring_head;       // next write position
static int ring_count;

void hr_algo_reset(void) {
    ring_head = 0;
    ring_count = 0;
}

void hr_algo_add_sample(uint32_t green) {
    ring[ring_head] = green;
    ring_head = (ring_head + 1) % HR_WINDOW;
    if (ring_count < HR_WINDOW) ring_count++;
}

int hr_algo_sample_count(void) {
    return ring_count;
}

// ---- Work buffers ----------------------------------------------------------
static float wx[HR_WINDOW];                 // window, then motion-bridged copy
static float wac[HR_WINDOW];                // band-passed signal
static float wtmp[HR_WINDOW + 2 * PADLEN];  // scratch (baseline, padded filter)
static float wsort[HR_WINDOW];              // scratch for medians/percentiles
static bool  wmotion[HR_WINDOW];
static bool  wflag[HR_WINDOW];
static int   wpeaks[HR_WINDOW / 2];
static int   worder[HR_WINDOW / 2];
static bool  wkeep[HR_WINDOW / 2];
static float wr[AC_LAG_HI + 2];

static int cmp_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

// np.percentile with linear interpolation. Sorts a copy in wsort.
static float percentile_sorted(const float *sorted, int n, float p) {
    float pos = p / 100.0f * (float)(n - 1);
    int i = (int)pos;
    if (i >= n - 1) return sorted[n - 1];
    float f = pos - (float)i;
    return sorted[i] + f * (sorted[i + 1] - sorted[i]);
}

static float median_of(const float *v, int n) {
    memcpy(wsort, v, (size_t)n * sizeof(float));
    qsort(wsort, (size_t)n, sizeof(float), cmp_float);
    return percentile_sorted(wsort, n, 50.0f);
}

// scipy.ndimage.median_filter(x, size, mode='nearest'), with a sliding
// sorted window (remove the outgoing sample, insert the incoming one).
static void running_median(const float *x, int n, float *out) {
    static float win[MEDIAN_SIZE];
    const int h = MEDIAN_SIZE / 2;
    #define AT(j) x[(j) < 0 ? 0 : ((j) >= n ? n - 1 : (j))]
    for (int j = 0; j < MEDIAN_SIZE; j++) win[j] = AT(j - h);
    qsort(win, MEDIAN_SIZE, sizeof(float), cmp_float);
    for (int i = 0; i < n; i++) {
        out[i] = win[h];
        if (i == n - 1) break;
        float gone = AT(i - h), add = AT(i + h + 1);
        int k = 0;
        while (k < MEDIAN_SIZE - 1 && win[k] != gone) k++;
        memmove(&win[k], &win[k + 1], (size_t)(MEDIAN_SIZE - 1 - k) * sizeof(float));
        int m = 0;
        while (m < MEDIAN_SIZE - 1 && win[m] < add) m++;
        memmove(&win[m + 1], &win[m], (size_t)(MEDIAN_SIZE - 1 - m) * sizeof(float));
        win[m] = add;
    }
    #undef AT
}

// One sosfilt pass over y[0..n-1] (step = +1 forwards, -1 backwards),
// starting from the steady state for input value x0.
static void sos_pass(float *y, int n, int step) {
    int i = step > 0 ? 0 : n - 1;
    float x0 = y[i];
    float z[2][2];
    for (int s = 0; s < 2; s++) {
        z[s][0] = ZI[s][0] * x0;
        z[s][1] = ZI[s][1] * x0;
    }
    for (int c = 0; c < n; c++, i += step) {
        float v = y[i];
        for (int s = 0; s < 2; s++) {
            const float *q = SOS[s];
            float out = q[0] * v + z[s][0];
            z[s][0] = q[1] * v - q[3] * out + z[s][1];
            z[s][1] = q[2] * v - q[4] * out;
            v = out;
        }
        y[i] = v;
    }
}

// scipy.signal.sosfiltfilt(SOS, x) with padtype='odd', padlen=PADLEN.
static void filtfilt(const float *x, int n, float *out) {
    float *e = wtmp;
    for (int k = 0; k < PADLEN; k++) {
        e[k] = 2 * x[0] - x[PADLEN - k];
        e[PADLEN + n + k] = 2 * x[n - 1] - x[n - 2 - k];
    }
    memcpy(&e[PADLEN], x, (size_t)n * sizeof(float));
    sos_pass(e, n + 2 * PADLEN, +1);
    sos_pass(e, n + 2 * PADLEN, -1);
    memcpy(out, &e[PADLEN], (size_t)n * sizeof(float));
}

// _filter_and_blank(): find motion on the raw signal, bridge it with a
// straight line, then band-pass. Fills wx (bridged), wac and wmotion.
static void filter_and_blank(int n) {
    float *base = wtmp;     // reused: filtfilt runs after we are done with it
    float *resid = wac;     // reused: overwritten by filtfilt at the end
    running_median(wx, n, base);
    for (int i = 0; i < n; i++) resid[i] = wx[i] - base[i];
    float med = median_of(resid, n);
    for (int i = 0; i < n; i++) base[i] = fabsf(resid[i] - med);
    float scale = 1.4826f * median_of(base, n) + 1e-9f;

    bool any = false;
    for (int i = 0; i < n; i++) {
        wflag[i] = fabsf(resid[i]) > MOTION_K * scale;
        any |= wflag[i];
    }
    memset(wmotion, 0, (size_t)n);
    if (any) {
        // Widen each burst by MOTION_PAD samples either side.
        for (int i = 0; i < n; i++) {
            if (!wflag[i]) continue;
            int a = i - MOTION_PAD < 0 ? 0 : i - MOTION_PAD;
            int b = i + MOTION_PAD >= n ? n - 1 : i + MOTION_PAD;
            for (int j = a; j <= b; j++) wmotion[j] = true;
        }
        int good = 0;
        for (int i = 0; i < n; i++) good += !wmotion[i];
        if (good > 2) {
            // np.interp: straight line between the good samples either side
            // (holding the edge value before the first / after the last).
            int prev = -1;
            for (int i = 0; i < n; i++) {
                if (!wmotion[i]) { prev = i; continue; }
                int next = i;
                while (next < n && wmotion[next]) next++;
                for (int j = i; j < next; j++) {
                    if (prev < 0)       wx[j] = wx[next];
                    else if (next >= n) wx[j] = wx[prev];
                    else wx[j] = wx[prev] + (wx[next] - wx[prev]) *
                                 (float)(j - prev) / (float)(next - prev);
                }
                i = next - 1;
            }
        }
    }
    float mean = 0;
    for (int i = 0; i < n; i++) mean += wx[i];
    mean /= (float)n;
    for (int i = 0; i < n; i++) wsort[i] = wx[i] - mean;
    filtfilt(wsort, n, wac);
    for (int i = 0; i < n; i++) if (wmotion[i]) wac[i] = 0.0f;
}

// Robust pulse size: 95th - 5th percentile of the non-motion samples.
static float clean_amplitude(int n) {
    int m = 0;
    for (int i = 0; i < n; i++) if (!wmotion[i]) wsort[m++] = wac[i];
    if (m == 0) { memcpy(wsort, wac, (size_t)n * sizeof(float)); m = n; }
    qsort(wsort, (size_t)m, sizeof(float), cmp_float);
    return percentile_sorted(wsort, m, 95.0f) - percentile_sorted(wsort, m, 5.0f);
}

static int cmp_peak_height(const void *a, const void *b) {
    int i = *(const int *)a, j = *(const int *)b;
    float x = wac[wpeaks[i]], y = wac[wpeaks[j]];
    if (x != y) return (x > y) - (x < y);
    return (i > j) - (i < j);           // stable for ties
}

// scipy.signal.find_peaks(ac, distance=MIN_IBI_SAMPLES, prominence=pmin).
// Returns the number of peaks written to wpeaks.
static int find_peaks(int n, float pmin) {
    const float *x = wac;
    int np_ = 0;
    // Local maxima, plateaus reported at their middle.
    for (int i = 1; i < n - 1; i++) {
        if (x[i - 1] < x[i]) {
            int ahead = i + 1;
            while (ahead < n - 1 && x[ahead] == x[i]) ahead++;
            if (x[ahead] < x[i]) {
                wpeaks[np_++] = (i + ahead - 1) / 2;
                if (np_ >= HR_WINDOW / 2) break;
                i = ahead - 1;   // skip the plateau; loop's i++ lands on ahead (as scipy)
            }
        }
    }
    // Distance: keep the highest peaks, drop neighbours closer than distance.
    for (int i = 0; i < np_; i++) { worder[i] = i; wkeep[i] = true; }
    qsort(worder, (size_t)np_, sizeof(int), cmp_peak_height);
    for (int o = np_ - 1; o >= 0; o--) {
        int j = worder[o];
        if (!wkeep[j]) continue;
        for (int k = j - 1; k >= 0 && wpeaks[j] - wpeaks[k] < MIN_IBI_SAMPLES; k--) wkeep[k] = false;
        for (int k = j + 1; k < np_ && wpeaks[k] - wpeaks[j] < MIN_IBI_SAMPLES; k++) wkeep[k] = false;
    }
    // Prominence over the whole window.
    int out = 0;
    for (int i = 0; i < np_; i++) {
        if (!wkeep[i]) continue;
        int p = wpeaks[i];
        float h = x[p], left = h, right = h;
        for (int k = p; k >= 0 && x[k] <= h; k--) if (x[k] < left) left = x[k];
        for (int k = p; k < n && x[k] <= h; k++) if (x[k] < right) right = x[k];
        float prom = h - (left > right ? left : right);
        if (prom >= pmin) wpeaks[out++] = p;
    }
    return out;
}

// _peak_analysis(): BPM from beat-to-beat intervals, outliers dropped.
static float peak_bpm(int n, float amplitude) {
    float pmin = PROMINENCE_FRAC * amplitude;
    if (pmin < 3.0f) pmin = 3.0f;
    int np_ = find_peaks(n, pmin);
    int m = 0;
    for (int i = 0; i < np_; i++) {
        if (wpeaks[i] >= EDGE_START && wpeaks[i] < n - EDGE_END) wpeaks[m++] = wpeaks[i];
    }
    np_ = m;
    if (np_ < MIN_GOOD_INTERVALS + 1 || amplitude < MIN_AMPLITUDE
            || (n - wpeaks[np_ - 1]) > 2 * HR_FS) {
        return 0;
    }
    // Intervals (in samples) that don't span motion and are in range.
    int count = 0;
    for (int i = 0; i + 1 < np_; i++) {
        int a = wpeaks[i], b = wpeaks[i + 1];
        bool spans = false;
        for (int k = a; k < b; k++) if (wmotion[k]) { spans = true; break; }
        int d = b - a;
        if (spans || d < MIN_IBI_SAMPLES || d * MAX_IBI_DEN > MAX_IBI_NUM) continue;
        wsort[count++] = (float)d / HR_FS;
    }
    if (count < MIN_GOOD_INTERVALS) return 0;
    static float ibi[HR_WINDOW / 2];
    memcpy(ibi, wsort, (size_t)count * sizeof(float));
    float med = median_of(ibi, count);
    float sum = 0;
    int good = 0;
    for (int i = 0; i < count; i++) {
        if (fabsf(ibi[i] - med) <= OUTLIER_FRAC * med) { sum += ibi[i]; good++; }
    }
    if (good < MIN_GOOD_INTERVALS || (float)good < MIN_GOOD_RATIO * (float)count) return 0;
    return 60.0f / (sum / (float)good);
}

// autocorr_bpm(): BPM from the repeating period. *strength = 0..1.
static float autocorr_bpm(int n, float *strength) {
    const float *seg = &wac[EDGE_START];
    int m = n - EDGE_START - EDGE_END;
    *strength = 0;
    if (m < 2 * AC_LAG_HI) return 0;
    float mean = 0;
    for (int i = 0; i < m; i++) mean += seg[i];
    mean /= (float)m;
    float *s = wsort;
    for (int i = 0; i < m; i++) s[i] = seg[i] - mean;
    for (int k = 0; k <= AC_LAG_HI + 1; k++) {
        float acc = 0;
        for (int i = 0; i + k < m; i++) acc += s[i] * s[i + k];
        wr[k] = acc;
    }
    if (wr[0] <= 0) return 0;
    float r0 = wr[0];
    for (int k = 0; k <= AC_LAG_HI + 1; k++) wr[k] = wr[k] / r0 * (float)m / (float)(m - k);
    float best = wr[AC_LAG_LO];
    for (int k = AC_LAG_LO; k <= AC_LAG_HI; k++) if (wr[k] > best) best = wr[k];
    *strength = best;
    if (best < MIN_AUTOCORR) return 0;
    // Shortest lag that is nearly as strong (avoid picking 2x the period).
    for (int k = AC_LAG_LO; k <= AC_LAG_HI; k++) {
        if (wr[k] >= 0.85f * best && wr[k] >= wr[k - 1] && wr[k] >= wr[k + 1]) {
            float denom = wr[k - 1] - 2 * wr[k] + wr[k + 1];
            float shift = denom != 0 ? 0.5f * (wr[k - 1] - wr[k + 1]) / denom : 0.0f;
            *strength = wr[k];
            return 60.0f * HR_FS / ((float)k + shift);
        }
    }
    return 0;
}

// analyze(): combine the peak and autocorrelation estimates.
float hr_algo_analyze(const float *x, int n) {
    if (n > HR_WINDOW) { x += n - HR_WINDOW; n = HR_WINDOW; }
    if (n < 3 * HR_FS) return 0;
    int saturated = 0;
    for (int i = 0; i < n; i++) saturated += x[i] >= (float)SATURATION;

    memcpy(wx, x, (size_t)n * sizeof(float));
    filter_and_blank(n);

    float amplitude = clean_amplitude(n);
    float bpm_peaks = peak_bpm(n, amplitude);

    int moving = 0;
    for (int i = 0; i < n; i++) moving += wmotion[i];
    if ((float)saturated > MAX_SATURATED_FRAC * (float)n
            || (float)moving > MAX_MOTION_FRAC * (float)n
            || amplitude < MIN_AMPLITUDE) {
        return 0;
    }
    float strength;
    float bpm_ac = autocorr_bpm(n, &strength);
    if (bpm_peaks > 0 && bpm_ac > 0) {
        if (fabsf(bpm_peaks - bpm_ac) <= AGREE_FRAC * bpm_ac) return bpm_peaks;
        return strength >= 0.7f ? bpm_ac : 0;
    }
    if (bpm_ac > 0) return bpm_ac;
    return bpm_peaks;
}

float hr_algo_estimate(void) {
    static float window[HR_WINDOW];
    int start = (ring_head - ring_count + HR_WINDOW) % HR_WINDOW;
    for (int i = 0; i < ring_count; i++) window[i] = (float)ring[(start + i) % HR_WINDOW];
    return hr_algo_analyze(window, ring_count);
}

// ---- BpmTracker --------------------------------------------------------------
#define TRK_TAU_S       3.0
#define TRK_JUMP_FRAC   0.15
#define TRK_CONFIRM_S   3.0
#define TRK_ACQUIRE_S   1.5
#define TRK_HOLD_S      8.0
#define TRK_HYSTERESIS  0.8

void bpm_tracker_reset(bpm_tracker_t *t) {
    memset(t, 0, sizeof *t);
}

int bpm_tracker_update(bpm_tracker_t *t, float raw_f, double now) {
    double dt = 0.0;
    if (t->has_last_t) {
        dt = now - t->last_t;
        if (dt > 1.0) dt = 1.0;
    }
    t->has_last_t = true;
    t->last_t = now;

    if (raw_f > 0) {
        double raw = raw_f;
        if (t->has_value && fabs(raw - t->value) <= TRK_JUMP_FRAC * t->value) {
            t->has_candidate = false;
            t->value += (1.0 - exp(-dt / TRK_TAU_S)) * (raw - t->value);
            t->last_good = now;
        } else {
            // Not locked yet, or the reading disagrees: track it as a candidate.
            if (!t->has_candidate || fabs(raw - t->candidate) > TRK_JUMP_FRAC * t->candidate) {
                t->has_candidate = true;
                t->candidate = raw;
                t->candidate_since = now;
            } else {
                t->candidate += 0.3 * (raw - t->candidate);
            }
            double needed = t->has_value ? TRK_CONFIRM_S : TRK_ACQUIRE_S;
            if (now - t->candidate_since >= needed) {
                t->has_value = true;
                t->value = t->candidate;
                t->has_candidate = false;
                t->last_good = now;
            }
        }
    }

    if (t->has_value && now - t->last_good > TRK_HOLD_S) {
        bpm_tracker_reset(t);
        t->has_last_t = true;
        t->last_t = now;
    }

    if (!t->has_value) {
        t->shown = 0;
    } else if (t->shown == 0 || fabs(t->value - t->shown) >= TRK_HYSTERESIS) {
        t->shown = (int)nearbyint(t->value);   // round-half-even, like Python round()
    }
    return t->shown;
}
