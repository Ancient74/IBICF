import serial
import threading
import queue
from collections import deque
from matplotlib import pyplot as plt

BAUD_RATE = 115200
samples = queue.Queue()
stopEvent = threading.Event()


def readSerial():
    with serial.Serial("COM7", BAUD_RATE, timeout=1) as connection:
        while not stopEvent.is_set():
            line = connection.readline().decode("utf-8").rstrip()
            if not line:
                continue
            try:
                a = line
                samples.put((int(a)))
            except ValueError:
                pass


def updatePlot():
    MAX_POINTS = 200
    xs = deque(maxlen=MAX_POINTS)
    ys = deque(maxlen=MAX_POINTS)
    sampleIdx = 0

    plt.ion()
    fig, ax = plt.subplots()
    (lineRaw,) = ax.plot([], [], "-", label="Value")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Value")
    ax.legend(loc="upper right")

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        updated = False
        try:
            while True:
                v = samples.get_nowait()
                xs.append(sampleIdx)
                ys.append(v)
                sampleIdx += 1
                updated = True
        except queue.Empty:
            pass

        if updated:
            lineRaw.set_data(xs, ys)
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
