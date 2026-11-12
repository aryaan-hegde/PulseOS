"""Live MAX30101 plot. Run: python plot_heartrate.py --port COM5"""
import argparse
from collections import deque
import queue
import threading
import time
import numpy as np
from scipy.signal import butter, sosfiltfilt, find_peaks

FS = 100  # Must match sensor configuration: 100 sps, no FIFO averaging.

def analyze(samples):
    """Return AC waveform, peak indices and an optional window-average BPM."""
    x = np.asarray(samples, dtype=float)
    if len(x) < 1 * FS:
        return x - np.mean(x), np.array([], dtype=int), None
    # Remove large DC optical background and high-frequency noise.
    sos = butter(3, [0.5, 4.0], btype='bandpass', fs=FS, output='sos')
    ac = sosfiltfilt(sos, x)
    # Ignore filter edges and the newest 0.5 s (not enough future context).
    peaks, _ = find_peaks(ac, distance=int(FS * 0.30),
                          prominence=max(5.0, 0.65 * np.std(ac)))
    peaks = peaks[(peaks >= FS) & (peaks < len(x) - FS // 2)]
    bpm = None
    if len(peaks) >= 2:
        intervals = np.diff(peaks) / FS
        # Suppress estimates on clipping, missing pulses, irregular motion or stale peaks.
        valid = (np.all((intervals >= 0.30) & (intervals <= 1.5))
                 and np.std(intervals) / np.mean(intervals) < 0.50
                 and (len(x) - peaks[-1]) / FS < 2.0
                 and np.ptp(ac) > 20
                 and not np.any(x >= 262000))
        if valid:
            bpm = 60.0 / np.mean(intervals)
    return ac, peaks, bpm

def self_test():
    t = np.arange(1200) / FS
    for expected in (60, 90, 120):
        x = 70000 + 500 * np.sin(2 * np.pi * expected / 60 * t)
        _, _, measured = analyze(x)
        assert measured is not None and abs(measured - expected) < 2, (expected, measured)
    assert analyze(np.full(1200, 70000))[2] is None
    assert analyze(np.full(1200, 262143))[2] is None
    print('Passed synthetic 60/90/120 BPM, flat and saturated signal checks.')

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
    fig.suptitle('MAX30101 — hold a fingertip still on the sensor')
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

    bpm_line, = bpm_ax.plot([], [], color="crimson")
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
            label = 'No fresh data — check connection'
        else:
            label = (f'Estimated average: {bpm:.1f} BPM (last 12 seconds)'
                     if bpm is not None else 'BPM: collecting / unstable signal — hold still')
        elapsed = now - plot_start

        bpm_times.append(elapsed)
        bpm_values.append(bpm if bpm is not None else np.nan)

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
