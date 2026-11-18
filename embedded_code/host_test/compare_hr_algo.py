"""Check that the C heart rate algorithm (src/hr_algo.c) matches the Python
reference (src/plot_heartrate.py) on the same signals.

Run from embedded_code/:
    python host_test/compare_hr_algo.py
    python host_test/compare_hr_algo.py --csv recording.csv   # also real data

Needs gcc on the PATH. Compiles host_test/hr_algo_host.c + src/hr_algo.c,
feeds both implementations identical windows and tracker inputs, and
reports any disagreement.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
import plot_heartrate as ref  # noqa: E402

FS = ref.FS
BPM_TOL = 1.0   # C (float32) vs Python (float64) may differ slightly


def build():
    # Build outside the repo (OneDrive/antivirus can block new .exe files there).
    exe = os.path.join(tempfile.gettempdir(),
                       'hr_algo_host.exe' if os.name == 'nt' else 'hr_algo_host')
    cmd = ['gcc', '-O2', '-Wall', '-Wextra', '-std=c11', '-o', exe,
           os.path.join(HERE, 'hr_algo_host.c'), os.path.join(SRC, 'hr_algo.c'), '-lm']
    # Put gcc's own folder first on PATH: Git Bash ships DLLs with the same
    # names, and if those load first gcc's cc1 exits silently with an error.
    env = dict(os.environ)
    gcc = shutil.which('gcc')
    if gcc:
        env['PATH'] = os.path.dirname(gcc) + os.pathsep + env.get('PATH', '')
    subprocess.run(cmd, check=True, env=env)
    return exe


def synthetic_windows(rng):
    """(name, samples) pairs covering clean, noisy, wrist-like and bad signals."""
    t = np.arange(1200) / FS
    out = []
    for bpm in (40, 45, 55, 60, 72, 90, 110, 120, 150, 170, 190):
        out.append((f'clean {bpm}', 70000 + 500 * np.sin(2 * np.pi * bpm / 60 * t)))
    for trial in range(40):
        bpm = rng.uniform(45, 170)
        f = bpm / 60
        amp = rng.uniform(5, 300)
        pulse = np.sin(2 * np.pi * f * t) + rng.uniform(0, 0.4) * np.sin(4 * np.pi * f * t + 0.5)
        x = (rng.uniform(20000, 200000) + amp * pulse
             + rng.uniform(0, amp / 2) * rng.standard_normal(len(t))
             + rng.uniform(0, 1000) * np.sin(2 * np.pi * rng.uniform(0.03, 0.2) * t))
        for _ in range(rng.integers(0, 3)):   # arm bumps
            a = rng.integers(0, 1100)
            x[a:a + rng.integers(10, 80)] += rng.uniform(-3000, 3000)
        out.append((f'random {trial} ({bpm:.0f} bpm, amp {amp:.0f})', x))
    out.append(('flat', np.full(1200, 70000.0)))
    out.append(('saturated', np.full(1200, 262143.0)))
    out.append(('noise', 70000 + 10 * rng.standard_normal(1200)))
    out.append(('short (4 s)', 70000 + 300 * np.sin(2 * np.pi * 1.2 * t[:400])))
    return out


def csv_windows(path):
    """Slide a 12 s window (every 0.5 s) over a recorded CSV of green samples."""
    data = np.loadtxt(path, delimiter=',', ndmin=2)
    green = data[:, 1] if data.shape[1] > 1 else data[:, 0]   # PPG,n,green,ir -> n,green,ir
    return [(f'{os.path.basename(path)} @ {end / FS:.1f}s', green[max(0, end - 1200):end])
            for end in range(300, len(green) + 1, FS // 2)]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--csv', action='append', default=[],
                        help='recorded samples (from plot_heartrate.py --record)')
    parser.add_argument('--exe', help='use an already-built hr_algo_host instead of compiling')
    args = parser.parse_args()

    exe = args.exe or build()
    proc = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def ask(cmd):
        proc.stdin.write(cmd + '\n')
        proc.stdin.flush()
        return proc.stdout.readline().strip()

    rng = np.random.default_rng(1)
    windows = synthetic_windows(rng)
    for path in args.csv:
        windows += csv_windows(path)

    failures = 0
    agree_some = agree_none = 0
    for name, x in windows:
        x = np.asarray(x, dtype=float)
        py = ref.analyze(x)[2]
        c = float(ask('W,' + ','.join(f'{v:.3f}' for v in x)))
        c = None if c == 0 else c
        if py is None and c is None:
            agree_none += 1
        elif py is not None and c is not None and abs(py - c) <= BPM_TOL:
            agree_some += 1
        else:
            failures += 1
            print(f'MISMATCH {name}: python={py} c={c}')

    # Tracker: same scenario as plot_heartrate.self_test plus a noisy stream.
    ask('R')
    tracker = ref.BpmTracker()
    now = 0.0
    stream = ([72.0] * 30 + [140.0] + [None] * 40 + [None] * 100 + [72.0] * 20 + [95.0] * 50
              + list(rng.choice([None, 80.0, 81.5, 79.0, 120.0], 200)))
    for raw in stream:
        now += 0.1
        py = tracker.update(raw, now)
        c = int(ask(f'T,{0 if raw is None else raw},{now:.6f}'))
        if (py or 0) != c:
            failures += 1
            print(f'TRACKER MISMATCH t={now:.1f} raw={raw}: python={py} c={c}')

    proc.stdin.close()
    proc.wait()
    print(f'Windows: {agree_some} agreed on a BPM, {agree_none} agreed on "no reading", '
          f'{failures} mismatches. Tracker: {len(stream)} steps checked.')
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
