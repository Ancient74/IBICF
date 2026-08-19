import serial
import threading
import queue
import time
from collections import deque

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


PORT = "COM7"
BAUD_RATE = 115200

MAX_POINTS = 2000
CALIBRATION_PERIOD = 5.0

SENSORS = {
    "acc": ("Accelerometer", "acc"),
    "gyro": ("Gyroscope", "gyro"),
    "magnet": ("Magnetometer", "magnet"),
}

COMPONENTS = ("x", "y", "z")

samples = queue.Queue()
stopEvent = threading.Event()


def parseLine(line):
    if ":" not in line:
        return None

    prefix, rest = line.split(":", 1)
    prefix = prefix.strip()

    if prefix not in SENSORS:
        return None

    try:
        values = tuple(float(v) / 1000.0 for v in rest.split())
    except ValueError:
        return None

    if len(values) != 3:
        return None

    return prefix, values


def fitSphere(points):
    if len(points) < 50:
        return np.zeros(3), 1.0

    P = np.asarray(points)

    A = np.column_stack([
        2.0 * P[:, 0],
        2.0 * P[:, 1],
        2.0 * P[:, 2],
        np.ones(len(P))
    ])

    b = np.sum(P * P, axis=1)

    try:
        solution, *_ = np.linalg.lstsq(A, b, rcond=None)

        center = solution[:3]

        radius = np.sqrt(
            solution[3]
            + center[0] ** 2
            + center[1] ** 2
            + center[2] ** 2
        )

        return center, radius

    except np.linalg.LinAlgError:
        return np.zeros(3), 1.0


def readSerial():
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as connection:
        while not stopEvent.is_set():
            line = connection.readline().decode(
                "utf-8",
                errors="replace"
            ).rstrip()

            if not line:
                continue

            parsed = parseLine(line)

            if parsed is not None:
                samples.put(parsed)


def updatePlot():
    state = {
        prefix: {
            "idx": 0,
            "xs": deque(maxlen=MAX_POINTS),
            "ys": {c: deque(maxlen=MAX_POINTS) for c in COMPONENTS},
        }
        for prefix in SENSORS
    }

    accRaw = deque(maxlen=MAX_POINTS)

    accBias = np.zeros(3)
    accRadius = 1.0

    lastCalibration = time.time()

    plt.ion()

    # -------------------------
    # Time series plots
    # -------------------------

    fig, axes = plt.subplots(
        len(SENSORS),
        1,
        figsize=(10, 8),
        squeeze=False
    )

    axes = axes.ravel()

    lines = {}

    for ax, (prefix, (title, ylabel)) in zip(
            axes,
            SENSORS.items()):

        lines[prefix] = {
            c: ax.plot([], [], label=c)[0]
            for c in COMPONENTS
        }

        ax.set_title(title)
        ax.set_xlabel("Sample")
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")

    fig.tight_layout()

    # -------------------------
    # 3D accelerometer plot
    # -------------------------

    fig3d = plt.figure(figsize=(8, 8))
    ax3d = fig3d.add_subplot(111, projection='3d')

    rawScatter = ax3d.scatter(
        [],
        [],
        [],
        s=3,
        label="raw"
    )

    calScatter = ax3d.scatter(
        [],
        [],
        [],
        s=3,
        label="calibrated"
    )

    ax3d.set_title("Accelerometer Calibration")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.legend()

    while (
        plt.fignum_exists(fig.number)
        and plt.fignum_exists(fig3d.number)
        and not stopEvent.is_set()
    ):
        touched = set()

        try:
            while True:
                prefix, values = samples.get_nowait()

                s = state[prefix]

                s["xs"].append(s["idx"])

                for c, v in zip(COMPONENTS, values):
                    s["ys"][c].append(v)

                s["idx"] += 1

                if prefix == "acc":
                    accRaw.append(values)

                touched.add(prefix)

        except queue.Empty:
            pass

        # -------------------------
        # Update timeseries
        # -------------------------

        for prefix in touched:
            s = state[prefix]

            for c in COMPONENTS:
                lines[prefix][c].set_data(
                    s["xs"],
                    s["ys"][c]
                )

            ax = lines[prefix]["x"].axes
            ax.relim()
            ax.autoscale_view()

        # -------------------------
        # Recompute calibration
        # -------------------------

        now = time.time()

        if (
            len(accRaw) > 50
            and now - lastCalibration > CALIBRATION_PERIOD
        ):
            accBias, accRadius = fitSphere(accRaw)

            print(
                f"ACC bias:"
                f" x={accBias[0]:.4f}"
                f" y={accBias[1]:.4f}"
                f" z={accBias[2]:.4f}"
                f" radius={accRadius:.4f}"
            )

            lastCalibration = now

        # -------------------------
        # Update 3D cloud
        # -------------------------

        if len(accRaw) > 0:
            P = np.asarray(accRaw)

            calibrated = P - accBias

            rawScatter._offsets3d = (
                P[:, 0],
                P[:, 1],
                P[:, 2]
            )

            calScatter._offsets3d = (
                calibrated[:, 0],
                calibrated[:, 1],
                calibrated[:, 2]
            )

            allPoints = np.vstack((P, calibrated))

            mins = allPoints.min(axis=0)
            maxs = allPoints.max(axis=0)

            center = (mins + maxs) / 2.0

            radius = np.max(maxs - mins) / 2.0
            radius = max(radius, 1.0)

            ax3d.set_xlim(
                center[0] - radius,
                center[0] + radius
            )

            ax3d.set_ylim(
                center[1] - radius,
                center[1] + radius
            )

            ax3d.set_zlim(
                center[2] - radius,
                center[2] + radius
            )

        plt.pause(0.05)

    stopEvent.set()


def main():
    worker = threading.Thread(
        target=readSerial,
        daemon=True
    )

    worker.start()

    try:
        updatePlot()
    finally:
        stopEvent.set()


if __name__ == "__main__":
    main()
