"""Live MAX30101 plot. Run: python plot_heartrate.py --port COM5"""
import argparse
from collections import deque
import queue
import threading
import time
import numpy as np
from scipy.signal import butter, sosfiltfilt, find_peaks
from scipy.ndimage import median_filter

FS = 100  # Must match sensor configuration: 100 sps, no FIFO averaging.

# --- Per-window estimator tuning -------------------------------------------
BAND_HZ = (0.7, 3.5)        # 42-210 BPM passband
MIN_IBI = 60.0 / 200        # shortest accepted beat interval (s)
MAX_IBI = 60.0 / 35         # longest accepted beat interval (s)
PROMINENCE_FRAC = 0.30      # peak must rise >= 30% of the robust pulse amplitude
OUTLIER_FRAC = 0.25         # intervals > 25% from the median are discarded
MIN_GOOD_INTERVALS = 3
MIN_GOOD_RATIO = 0.6        # at least 60% of intervals must agree
MIN_AMPLITUDE = 8           # robust pulse amplitude floor (ADC counts); wrist pulses are small
SATURATION = 262000
MAX_SATURATED_FRAC = 0.05
MIN_AUTOCORR = 0.5          # periodicity strength needed to trust the autocorrelation estimate
AGREE_FRAC = 0.10           # peak and autocorrelation estimates "agree" within 10%
MOTION_K = 6.0              # raw residual > 6x its typical size = movement, not pulse
MOTION_PAD_S = 0.25         # also blank this long either side of a movement burst
MAX_MOTION_FRAC = 0.4       # give up if more than 40% of the window is motion


def autocorr_bpm(ac):
    """Estimate BPM from the repeating period of the signal.

    More robust than peak picking on the wrist, where pulses are small and
    rounded. Returns (bpm, strength 0..1) or (None, 0).
    """
    seg = ac[FS // 2: len(ac) - FS // 4]
    seg = seg - np.mean(seg)
    n = len(seg)
    lo, hi = int(MIN_IBI * FS), int(MAX_IBI * FS)
    if n < 2 * hi:
        return None, 0.0
    r = np.correlate(seg, seg, mode='full')[n - 1:n + hi + 1]
    if r[0] <= 0:
        return None, 0.0
    r = r / r[0] * n / (n - np.arange(len(r)))   # normalise, correct for shrinking overlap
    best = np.max(r[lo:hi + 1])
    if best < MIN_AUTOCORR:
        return None, float(best)
    # Peaks also appear at 2x, 3x the period: take the shortest lag that is nearly as strong.
    for k in range(lo, hi + 1):
        if r[k] >= 0.85 * best and r[k] >= r[k - 1] and r[k] >= r[k + 1]:
            denom = r[k - 1] - 2 * r[k] + r[k + 1]
            shift = 0.5 * (r[k - 1] - r[k + 1]) / denom if denom != 0 else 0.0  # sub-sample peak
            return 60.0 * FS / (k + shift), float(r[k])
    return None, float(best)


def analyze(samples):
    """Return AC waveform, peak indices and an optional BPM for this window.

    Motion bursts are blanked first, then two estimates are combined:
    beat-to-beat peak intervals (outliers dropped) and autocorrelation (period
    of the whole window). If both agree the peak estimate is used; if only one
    is available it is used when trustworthy.
    """
    x = np.asarray(samples, dtype=float)
    if len(x) < 3 * FS:
        return x - np.mean(x), np.array([], dtype=int), None
    ac, motion = _filter_and_blank(x)
    ac, peaks, bpm_peaks = _peak_analysis(x, ac, motion)
    if (np.mean(x >= SATURATION) > MAX_SATURATED_FRAC
            or np.mean(motion) > MAX_MOTION_FRAC):
        return ac, peaks, None
    lo, hi = np.percentile(ac[~motion], [5, 95])
    if hi - lo < MIN_AMPLITUDE:
        return ac, peaks, None
    bpm_ac, strength = autocorr_bpm(ac)
    if bpm_peaks is not None and bpm_ac is not None:
        if abs(bpm_peaks - bpm_ac) <= AGREE_FRAC * bpm_ac:
            return ac, peaks, bpm_peaks
        return ac, peaks, (bpm_ac if strength >= 0.7 else None)
    if bpm_ac is not None:
        return ac, peaks, bpm_ac
    return ac, peaks, bpm_peaks


def _filter_and_blank(x):
    """Remove motion bursts from the raw signal, then band-pass it.

    Motion is found on the raw signal (before filtering, so the filter cannot
    smear it out): the signal minus a 1.5 s running median should only contain
    pulse + noise, so anything far beyond its typical size is movement. Those
    samples are bridged with a straight line before filtering and marked so
    later steps ignore them. Returns (ac, motion_mask).
    """
    baseline = median_filter(x, size=int(1.5 * FS) | 1, mode='nearest')
    resid = x - baseline
    scale = 1.4826 * np.median(np.abs(resid - np.median(resid))) + 1e-9
    motion = np.abs(resid) > MOTION_K * scale
    if motion.any():
        pad = int(MOTION_PAD_S * FS)
        motion = np.convolve(motion, np.ones(2 * pad + 1), mode='same') > 0
        if (~motion).sum() > 2:
            idx = np.arange(len(x))
            x = x.copy()
            x[motion] = np.interp(idx[motion], idx[~motion], x[~motion])
    # Remove large DC optical background and high-frequency noise.
    sos = butter(2, BAND_HZ, btype='bandpass', fs=FS, output='sos')
    ac = sosfiltfilt(sos, x - np.mean(x))
    ac[motion] = 0.0
    return ac, motion


def _peak_analysis(x, ac, motion):
    """Peak-interval estimate: a few bad beats are dropped as outliers."""
    clean = ac[~motion] if (~motion).any() else ac
    lo, hi = np.percentile(clean, [5, 95])
    amplitude = hi - lo
    peaks, _ = find_peaks(ac, distance=int(FS * MIN_IBI),
                          prominence=max(3.0, PROMINENCE_FRAC * amplitude))
    # Ignore filter edges and the newest 0.25 s (not enough future context).
    peaks = peaks[(peaks >= FS // 2) & (peaks < len(x) - FS // 4)]

    if (len(peaks) < MIN_GOOD_INTERVALS + 1
            or amplitude < MIN_AMPLITUDE
            or (len(x) - peaks[-1]) / FS > 2.0):   # watch removed / stale
        return ac, peaks, None

    # Drop intervals that span a blanked motion burst (a beat is hidden in there).
    spans_motion = np.array([motion[a:b].any() for a, b in zip(peaks[:-1], peaks[1:])])
    ibi = (np.diff(peaks) / FS)[~spans_motion]
    ibi = ibi[(ibi >= MIN_IBI) & (ibi <= MAX_IBI)]
    if len(ibi) < MIN_GOOD_INTERVALS:
        return ac, peaks, None
    median = np.median(ibi)
    good = ibi[np.abs(ibi - median) <= OUTLIER_FRAC * median]
    if len(good) < MIN_GOOD_INTERVALS or len(good) < MIN_GOOD_RATIO * len(ibi):
        return ac, peaks, None
    return ac, peaks, 60.0 / np.mean(good)


class BpmTracker:
    """Turns noisy per-window estimates into a steady number for a display.

    - Smooths with a time-constant EMA so the value drifts, not jumps.
    - A new reading that disagrees by > JUMP_FRAC must persist for
      CONFIRM_S before it is accepted (real HR change vs. glitch).
    - Keeps showing the last value for HOLD_S after the signal is lost.
    - Integer output only changes when the smoothed value moves >= HYSTERESIS.
    Kept deliberately simple so it ports directly to C on the RP2350.
    """
    TAU_S = 3.0
    JUMP_FRAC = 0.15
    CONFIRM_S = 3.0
    ACQUIRE_S = 1.5
    HOLD_S = 8.0       # wrist: arm movement causes longer dropouts
    HYSTERESIS = 0.8

    def __init__(self):
        self.reset()

    def reset(self):
        self.value = None          # smoothed float BPM
        self.shown = None          # integer BPM for the display
        self.last_t = None
        self.last_good = None
        self.candidate = None
        self.candidate_since = None

    def update(self, raw, now):
        dt = 0.0 if self.last_t is None else min(now - self.last_t, 1.0)
        self.last_t = now

        if raw is not None:
            if self.value is not None and abs(raw - self.value) <= self.JUMP_FRAC * self.value:
                self.candidate = None
                self.value += (1 - np.exp(-dt / self.TAU_S)) * (raw - self.value)
                self.last_good = now
            else:
                # Not locked yet, or reading disagrees: track it as a candidate.
                if self.candidate is None or abs(raw - self.candidate) > self.JUMP_FRAC * self.candidate:
                    self.candidate, self.candidate_since = raw, now
                else:
                    self.candidate += 0.3 * (raw - self.candidate)
                needed = self.ACQUIRE_S if self.value is None else self.CONFIRM_S
                if now - self.candidate_since >= needed:
                    self.value, self.candidate = self.candidate, None
                    self.last_good = now

        if self.value is not None and now - self.last_good > self.HOLD_S:
            self.reset()
            self.last_t = now

        if self.value is None:
            self.shown = None
        elif self.shown is None or abs(self.value - self.shown) >= self.HYSTERESIS:
            self.shown = int(round(self.value))
        return self.shown


def self_test():
    rng = np.random.default_rng(0)
    t = np.arange(1200) / FS
    for expected in (45, 60, 90, 120, 170):
        x = 70000 + 500 * np.sin(2 * np.pi * expected / 60 * t)
        _, _, measured = analyze(x)
        assert measured is not None and abs(measured - expected) < 2, (expected, measured)
    assert analyze(np.full(1200, 70000))[2] is None
    assert analyze(np.full(1200, 262143))[2] is None

    # Realistic-ish 72 BPM pulse + noise + baseline wander + one motion spike.
    pulse = np.sin(2 * np.pi * 1.2 * t) + 0.3 * np.sin(2 * np.pi * 2.4 * t + 1)
    x = 70000 + 300 * pulse + 40 * rng.standard_normal(len(t)) + 800 * np.sin(2 * np.pi * 0.1 * t)
    x[600:640] += 6000
    _, _, measured = analyze(x)
    assert measured is not None and abs(measured - 72) < 3, measured

    # One missed beat (flattened) should be rejected as an outlier, not kill the reading.
    x = 70000 + 300 * np.sin(2 * np.pi * 1.2 * t)
    beat = int(FS / 1.2)
    x[5 * beat:6 * beat] = 70000
    _, _, measured = analyze(x)
    assert measured is not None and abs(measured - 72) < 3, measured

    # Wrist-like: tiny rounded pulse (~30 counts) buried in noise, drift and two arm bumps.
    for expected in (55, 72, 110):
        f = expected / 60
        pulse = np.sin(2 * np.pi * f * t) + 0.15 * np.sin(2 * np.pi * 2 * f * t + 0.5)
        x = (70000 + 15 * pulse + 6 * rng.standard_normal(len(t))
             + 400 * np.sin(2 * np.pi * 0.08 * t))
        x[300:350] += 1500
        x[900:930] -= 1000
        _, _, measured = analyze(x)
        assert measured is not None and abs(measured - expected) < 3, (expected, measured)

    # Pure noise must not produce a BPM.
    assert analyze(70000 + 10 * rng.standard_normal(len(t)))[2] is None

    # Tracker: ignores single glitches, holds through dropouts, follows real changes.
    tr = BpmTracker()
    now = 0.0
    for _ in range(30):
        now += 0.1; tr.update(72.0, now)
    assert tr.shown == 72
    now += 0.1; assert tr.update(140.0, now) == 72          # glitch ignored
    for _ in range(40):
        now += 0.1; tr.update(None, now)
    assert tr.shown == 72                                    # held through 4 s dropout
    for _ in range(100):
        now += 0.1; tr.update(None, now)
    assert tr.shown is None                                  # cleared after hold
    for _ in range(20):
        now += 0.1; tr.update(72.0, now)
    for _ in range(50):
        now += 0.1; tr.update(95.0, now)
    assert tr.shown == 95                                    # sustained change accepted
    print('Passed synthetic BPM, noise/spike, missed-beat and tracker checks.')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Serial port, e.g. COM5 or /dev/ttyACM0')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    import serial
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    if not args.port:
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(port.device, port.description)
        parser.error('Pass --port with one of the ports listed above.')
    device = serial.Serial(args.port, 115200, timeout=0.2)
    events = queue.Queue(maxsize=5000)
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                line = device.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('PPG,'):
                    fields = line.split(',')
                    item = tuple(int(v) for v in fields[1:])
                    if len(item) != 3:
                        continue
                    try:
                        events.put_nowait(item)
                    except queue.Full:
                        # A sequence gap causes the plot to reset rather than bridge lost data.
                        pass
                else:
                    print(line)
                    if line.startswith('[HR]'):
                        try:
                            events.put_nowait(None)
                        except queue.Full:
                            pass
            except ValueError:
                continue
            except serial.SerialException as exc:
                print('Serial error:', exc)
                stop.set()

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    indices, red, ir = (deque(maxlen=1200) for _ in range(3))
    fig, (raw_ax, pulse_ax, bpm_ax) = plt.subplots(3, 1, figsize=(11, 9))
    fig.suptitle('MAX30101 — wear snugly on the wrist and keep the arm still')
    red_line, = raw_ax.plot([], [], color='tomato', label='Green')
    ir_line, = raw_ax.plot([], [], color='purple', label='IR')
    pulse_line, = pulse_ax.plot([], [], color='teal', label='Filtered green pulse')
    peak_line, = pulse_ax.plot([], [], 'o', color='orange', label='Detected peaks')
    raw_ax.set_ylabel('Raw ADC counts')
    pulse_ax.set_ylabel('Pulse amplitude (ADC counts)')
    pulse_ax.set_xlabel('Seconds relative to newest sample')
    for ax in (raw_ax, pulse_ax):
        ax.legend(loc='upper left')
        ax.grid(alpha=0.25)
    state = {'last_rx': 0.0, 'last_print': 0.0}
    bpm_times = deque(maxlen=600)
    bpm_values = deque(maxlen=600)
    plot_start = time.monotonic()

    tracker = BpmTracker()
    raw_bpm_values = deque(maxlen=600)
    raw_bpm_line, = bpm_ax.plot([], [], color="crimson", alpha=0.3, lw=1, label="Per-window estimate")
    bpm_line, = bpm_ax.plot([], [], color="crimson", lw=2.5, label="Displayed (smoothed)")
    bpm_ax.legend(loc="upper left")
    bpm_ax.set_ylabel("Heart rate (BPM)")
    bpm_ax.set_xlabel("Elapsed time (seconds)")
    bpm_ax.grid(alpha=0.25)

    def update(_):
        while True:
            try:
                item = events.get_nowait()
            except queue.Empty:
                break
            if item is None or (indices and item[0] != indices[-1] + 1):
                indices.clear(); red.clear(); ir.clear()
            if item is None:
                continue
            n, r, i = item
            indices.append(n); red.append(r); ir.append(i)
            state['last_rx'] = time.monotonic()
        now = time.monotonic()
        if not indices:
            pulse_ax.set_title('Waiting for PPG samples…')
            return
        t = (np.asarray(indices) - indices[-1]) / FS
        ac, peaks, bpm = analyze(red)  # First channel now contains green
        red_line.set_data(t, red); ir_line.set_data(t, ir)
        pulse_line.set_data(t, ac); peak_line.set_data(t[peaks], ac[peaks])
        if now - state['last_rx'] > 2 or stop.is_set():
            bpm = None
            tracker.reset()
            shown = None
            label = 'No fresh data — check connection'
        else:
            shown = tracker.update(bpm, now)
            if shown is None:
                label = 'BPM: -- (acquiring — hold still)'
            elif bpm is None:
                label = f'BPM: {shown} (holding — weak signal)'
            else:
                label = f'BPM: {shown}   (window estimate {bpm:.1f})'
        elapsed = now - plot_start

        bpm_times.append(elapsed)
        bpm_values.append(shown if shown is not None else np.nan)
        raw_bpm_values.append(bpm if bpm is not None else np.nan)

        raw_bpm_line.set_data(bpm_times, raw_bpm_values)
        bpm_line.set_data(bpm_times, bpm_values)
        bpm_ax.relim()
        bpm_ax.autoscale_view()
        bpm_ax.set_xlim(max(0, elapsed - 60), max(10, elapsed))

        pulse_ax.set_title(label)
        if now - state['last_print'] >= 2:
            print(label)
            state['last_print'] = now
        for ax in (raw_ax, pulse_ax):
            ax.relim(); ax.autoscale_view()
            ax.set_xlim(-12, 0)

    animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        stop.set()
        thread.join(timeout=1)
        device.close()

if __name__ == '__main__':
    main()
