import serial
import threading
import queue
from collections import deque
from matplotlib import pyplot as plt

PORT = "COM7"
BAUD_RATE = 115200
MAX_POINTS = 2000

# Each printf line looks like "<prefix>: <x> <y> <z>".
# Maps the serial prefix -> (subplot title, y-axis label).
SENSORS = {
    "psi": ("psi", "psi"),
   # "gyro": ("Gyroscope", "gyro"),
    #"magnet": ("Magnetometer", "magnet"),
}
COMPONENTS = ("x")

samples = queue.Queue()
stopEvent = threading.Event()


def parseLine(line):
    """Return (prefix, (x, y, z)) for a valid sensor line, else None."""
    if ":" not in line:
        return None
    prefix, rest = line.split(":", 1)
    prefix = prefix.strip()
    if prefix not in SENSORS:
        return None
    try:
        values = tuple(float(v)/1000.0 for v in rest.split())
    except ValueError:
        return None
    if len(values) != len(COMPONENTS):
        return None
    return prefix, values


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
    # Per-sensor state: independent sample index (sensors run at different
    # rates), and one deque per x/y/z component.
    state = {
        prefix: {
            "idx": 0,
            "xs": deque(maxlen=MAX_POINTS),
            "ys": {c: deque(maxlen=MAX_POINTS) for c in COMPONENTS},
        }
        for prefix in SENSORS
    }

    plt.ion()
    fig, axes = plt.subplots(len(SENSORS), 1, figsize=(9, 8), squeeze=False)
    axes = axes.ravel()

    lines = {}
    for ax, (prefix, (title, ylabel)) in zip(axes, SENSORS.items()):
        lines[prefix] = {
            c: ax.plot([], [], "-", label=c)[0] for c in COMPONENTS
        }
        ax.set_title(title)
        ax.set_xlabel("Sample")
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")
    fig.tight_layout()

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        touched = set()
        try:
            while True:
                prefix, values = samples.get_nowait()
                s = state[prefix]
                s["xs"].append(s["idx"])
                for c, v in zip(COMPONENTS, values):
                    s["ys"][c].append(v)
                s["idx"] += 1
                touched.add(prefix)
        except queue.Empty:
            pass

        for prefix in touched:
            s = state[prefix]
            for c in COMPONENTS:
                lines[prefix][c].set_data(s["xs"], s["ys"][c])
            ax = lines[prefix][COMPONENTS[0]].axes
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
