import serial
import threading
import queue
import numpy as np
from matplotlib import pyplot as plt

PORT = "COM7"
BAUD_RATE = 115200

ACC_PREFIX = "acc"
MAG_PREFIX = "magnet"

# Mag -> IMU body alignment (from align_calibration.py).
MAGNET_ALIGN = np.array([
    [-0.461761, -0.870776,  0.168897],
    [-0.863340,  0.397529, -0.310828],
    [ 0.203520, -0.289344, -0.935339],
])

# Hard/soft-iron correction. Leave as identity/zero if your firmware ALREADY
# applies the ellipsoid calibration before printing "magnet:". If it prints raw
# counts, paste your magnetBias / magnetMatrix from the sphere-cal script here.
MAG_BIAS = np.array([515.798978, -47.838570, -108.972277])
MAG_MATRIX = np.array([
    [ 1.241897, -0.050880, -0.096114],
    [-0.050880,  0.910678, -0.104038],
    [-0.096114, -0.104038,  1.028557],
])

HISTORY = 600          # samples shown in the scrolling plot
UNWRAP = True          # unwrap heading so a yaw reads as one smooth step

samples = queue.Queue()
stopEvent = threading.Event()


def parseLine(line):
    if ":" not in line:
        return None
    prefix, rest = line.split(":", 1)
    prefix = prefix.strip()
    try:
        values = tuple(float(v) for v in rest.split())
    except ValueError:
        return None
    if len(values) != 3:
        return None
    return prefix, values


def readSerial():
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as connection:
        while not stopEvent.is_set():
            line = connection.readline().decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            parsed = parseLine(line)
            if parsed is not None and parsed[0] in (ACC_PREFIX, MAG_PREFIX):
                samples.put(parsed)


def correctMag(raw):
    """Hard/soft-iron correction, then mag->body alignment."""
    corrected = MAG_MATRIX @ (np.asarray(raw, dtype=float) - MAG_BIAS)
    return MAGNET_ALIGN @ corrected


def tiltCompensatedHeading(acc, mag):
    """Heading of the field relative to body-forward, tilt-compensated by accel.

    Sign may be inverted relative to your yaw; what matters for the test is that
    the MAGNITUDE of the change tracks the physical yaw 1:1.
    """
    d = np.asarray(acc, dtype=float)
    d = d / np.linalg.norm(d)                 # down, body frame
    x = np.array([1.0, 0.0, 0.0])
    fwd = x - d * (d @ x)
    nf = np.linalg.norm(fwd)
    if nf < 1e-6:                             # nose pointing straight down
        x = np.array([0.0, 1.0, 0.0])
        fwd = x - d * (d @ x)
        nf = np.linalg.norm(fwd)
    fwd /= nf
    right = np.cross(d, fwd)
    return np.arctan2(mag @ right, mag @ fwd)   # radians


def run():
    headings = []      # radians
    mags = []          # field magnitude after correction
    latestAcc = None

    plt.ion()
    fig, (axH, axM) = plt.subplots(2, 1, figsize=(11, 7))
    fig.suptitle("yaw the board and watch the heading step by the same angle; "
                 "magnitude should stay flat")

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        try:
            while True:
                prefix, vals = samples.get_nowait()
                if prefix == ACC_PREFIX:
                    latestAcc = vals
                elif prefix == MAG_PREFIX and latestAcc is not None:
                    m = correctMag(vals)
                    headings.append(tiltCompensatedHeading(latestAcc, m))
                    mags.append(float(np.linalg.norm(m)))
        except queue.Empty:
            pass

        if headings:
            h = np.array(headings[-HISTORY:])
            mg = np.array(mags[-HISTORY:])
            disp = np.degrees(np.unwrap(h) if UNWRAP else h)

            axH.cla()
            axH.plot(disp, linewidth=1.2)
            axH.set_title(f"tilt-compensated heading: {disp[-1]:7.1f} deg")
            axH.set_ylabel("degrees")
            axH.grid(alpha=0.3)

            axM.cla()
            axM.plot(mg, linewidth=1.2, color="tab:orange")
            spread = 100.0 * (mg.max() - mg.min()) / max(mg.mean(), 1e-9)
            axM.set_title(f"field magnitude (spread {spread:.1f}% of mean - "
                          f"flat = clean field/calibration)")
            axM.set_ylabel("|field|")
            axM.set_xlabel("sample")
            axM.grid(alpha=0.3)

        plt.pause(0.05)

    stopEvent.set()


def main():
    worker = threading.Thread(target=readSerial, daemon=True)
    worker.start()
    run()


if __name__ == "__main__":
    main()
