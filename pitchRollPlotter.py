import serial
import threading
import queue
from collections import deque
from matplotlib import pyplot as plt

PORT = "COM7"
BAUD_RATE = 115200
MAX_POINTS = 2000

# Each line is now just "<roll> <pitch>" (whitespace separated, no prefix).
# If your firmware still prints integer milli-units (value * 1000), set
# SCALE = 0.001. If it prints plain floats, leave SCALE = 1.0.
SCALE = 0.001
SERIES = ("roll", "pitch")

samples = queue.Queue()
stopEvent = threading.Event()


def parseLine(line):
    """Return (roll, pitch) for a valid line, else None."""
    parts = line.split()
    if len(parts) != len(SERIES):
        return None
    try:
        return tuple(float(p) * SCALE for p in parts)
    except ValueError:
        return None


def readSerial():
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as connection:
        while not stopEvent.is_set():
            line = connection.readline().decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            parsed = parseLine(line)
            if parsed is not None:
                samples.put(parsed)


def updatePlot():
    idx = 0
    xs = deque(maxlen=MAX_POINTS)
    ys = {name: deque(maxlen=MAX_POINTS) for name in SERIES}

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 5))
    lines = {name: ax.plot([], [], "-", label=name)[0] for name in SERIES}
    ax.set_title("Orientation")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Angle (rad)")
    ax.legend(loc="upper right")
    fig.tight_layout()

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        touched = False
        try:
            while True:
                values = samples.get_nowait()
                xs.append(idx)
                for name, v in zip(SERIES, values):
                    ys[name].append(v)
                idx += 1
                touched = True
        except queue.Empty:
            pass

        if touched:
            for name in SERIES:
                lines[name].set_data(xs, ys[name])
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.05)

    stopEvent.set()


def main():
    worker = threading.Thread(target=readSerial, daemon=True)
    worker.start()
    updatePlot()


if __name__ == "__main__":
    main()
