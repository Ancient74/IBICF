#!/usr/bin/env python3
"""
quatPlane3D.py - live 3D aircraft attitude from a quaternion stream.

Reads a normalized quaternion over a serial port and rotates a 3D plane model
in real time, so you can see your IMU's orientation directly instead of staring
at two scrolling traces.

Each serial line must contain four whitespace- or comma-separated numbers.
By default they are read in w, x, y, z order (what the MPU-6050 DMP emits) -
change QUAT_ORDER if your firmware prints them in a different order.

Dependencies:
    pip install pyvista numpy pyserial
"""

import re
import time
import queue
import threading

import numpy as np
import pyvista as pv
import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PORT = "COM7"
BAUD_RATE = 115200

# Order the four numbers arrive in. The MPU-6050 DMP emits w, x, y, z.
QUAT_ORDER = "wxyz"

# If the firmware prints integer fixed-point values, set the divisor here:
#   plain floats already in [-1, 1] -> 1.0
#   DMP Q14 fixed point             -> 16384.0
#   milli-units (value * 1000)      -> 1000.0
SCALE = 1.0

# Flip to True if the model spins the opposite way to the real board, i.e. your
# stream encodes the world->body rotation rather than body->world.
INVERT = False

# Flip to True if roll and pitch are reversed (tilt right -> plane rolls left,
# pitch down -> plane pitches up) while yaw is correct. This is a 180 deg yaw
# alignment between the sensor frame and the model frame.
FLIP_ROLL_PITCH = True

# Zero the yaw at startup so the heading reads 0 at boot ("north" = whichever
# way the nose points when the first sample arrives). Roll and pitch are left
# alone, since those are referenced to gravity and are absolute already.
ZERO_YAW_AT_STARTUP = True

RENDER_HZ = 60  # how often to repaint the window
# ---------------------------------------------------------------------------

samples = queue.Queue()
stopEvent = threading.Event()

_ORDER_IDX = {name: QUAT_ORDER.index(name) for name in "wxyz"}
_SPLIT = re.compile(r"[,\s]+")


def parseLine(line):
    """Return a normalized (w, x, y, z) tuple for a valid line, else None."""
    parts = [p for p in _SPLIT.split(line.strip()) if p]
    if len(parts) != 4:
        return None
    try:
        vals = [float(p) * SCALE for p in parts]
    except ValueError:
        return None

    w = vals[_ORDER_IDX["w"]]
    x = vals[_ORDER_IDX["x"]]
    y = vals[_ORDER_IDX["y"]]
    z = vals[_ORDER_IDX["z"]]

    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm < 1e-6:
        return None
    w, x, y, z = (c / norm for c in (w, x, y, z))
    if INVERT:  # use the conjugate
        x, y, z = -x, -y, -z
    if FLIP_ROLL_PITCH:  # 180 deg yaw realignment -> inverts roll and pitch
        x, y = -x, -y
    return w, x, y, z


def readSerial():
    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=1) as connection:
            while not stopEvent.is_set():
                raw = connection.readline().decode("utf-8", errors="replace")
                if not raw:
                    continue
                parsed = parseLine(raw)
                if parsed is not None:
                    samples.put(parsed)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        stopEvent.set()


def matrixFromQuat(q):
    """Build a 4x4 model matrix from a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    m = np.eye(4)
    m[0, 0] = 1 - 2 * (y * y + z * z)
    m[0, 1] = 2 * (x * y - w * z)
    m[0, 2] = 2 * (x * z + w * y)
    m[1, 0] = 2 * (x * y + w * z)
    m[1, 1] = 1 - 2 * (x * x + z * z)
    m[1, 2] = 2 * (y * z - w * x)
    m[2, 0] = 2 * (x * z - w * y)
    m[2, 1] = 2 * (y * z + w * x)
    m[2, 2] = 1 - 2 * (x * x + y * y)
    return m


def quatToEulerDeg(q):
    """Roll, pitch, yaw in degrees (aerospace ZYX) for the on-screen readout."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def quatYawRad(q):
    """Just the yaw (heading) of a quaternion, in radians."""
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yawQuat(angle):
    """Quaternion for a rotation of `angle` radians about the world vertical."""
    half = angle / 2.0
    return (np.cos(half), 0.0, 0.0, np.sin(half))


def quatMultiply(a, b):
    """Hamilton product a (x) b (both as (w, x, y, z))."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def buildAircraft():
    """A simple, clearly oriented plane: nose +X, right wing +Y, up +Z.

    Returns (mesh, color) parts that all live in one body frame, so a single
    model matrix rotates them rigidly together.
    """
    fuselage = pv.Cylinder(center=(0, 0, 0), direction=(1, 0, 0),
                           radius=0.22, height=3.0, resolution=40)
    nose = pv.Cone(center=(1.65, 0, 0), direction=(1, 0, 0),
                   radius=0.22, height=0.6, resolution=40)
    body = fuselage.merge(nose)

    wings = pv.Box(bounds=(-0.35, 0.55, -2.1, 2.1, -0.04, 0.04))
    tailplane = pv.Box(bounds=(-1.55, -1.05, -0.95, 0.95, -0.04, 0.04))
    fin = pv.Box(bounds=(-1.55, -1.05, -0.04, 0.04, 0.0, 0.8))

    return [
        (body, "#c9ccd1"),       # silver fuselage + nose
        (wings, "#3a7bd5"),      # blue main wing
        (tailplane, "#3a7bd5"),  # blue horizontal stabilizer
        (fin, "#e23b3b"),        # red vertical fin -> unambiguous up/back
    ]


def main():
    print(f"Opening {PORT} @ {BAUD_RATE} ...")
    worker = threading.Thread(target=readSerial, daemon=True)
    worker.start()

    pl = pv.Plotter()
    pl.set_background("#10131a", top="#1d2230")
    pl.add_axes(line_width=3)  # fixed world reference frame, corner widget

    actors = []
    for mesh, color in buildAircraft():
        actors.append(pl.add_mesh(mesh, color=color, specular=0.4,
                                  specular_power=15))

    pl.add_text("waiting for data...", position="upper_left",
                font_size=11, name="hud", color="white")
    # Camera parked behind the tail (-X), looking forward toward the nose.
    pl.camera_position = [(-8, 0, 1.8), (0, 0, 0), (0, 0, 1)]

    # Non-blocking render loop on the main thread. More robust than a VTK
    # timer event, and any error surfaces in the console instead of being
    # silently swallowed by the interactor.
    pl.show(interactive_update=True, auto_close=False)

    seen = False
    yawOffset = None  # set once, from the first sample
    period = 1.0 / RENDER_HZ
    while not stopEvent.is_set():
        q = None
        try:
            while True:  # drain the queue, keep only the most recent reading
                q = samples.get_nowait()
        except queue.Empty:
            pass

        if q is not None:
            try:
                if not seen:
                    print("First sample received - rendering.")
                    seen = True
                if yawOffset is None and ZERO_YAW_AT_STARTUP:
                    # rotate out the startup heading about the world vertical
                    yawOffset = yawQuat(-quatYawRad(q))
                qd = quatMultiply(yawOffset, q) if yawOffset is not None else q

                m = matrixFromQuat(qd)
                for actor in actors:
                    actor.user_matrix = m
                r, p, yw = quatToEulerDeg(qd)
                pl.add_text(
                    f"q  w {qd[0]:+.3f}  x {qd[1]:+.3f}  y {qd[2]:+.3f}  z {qd[3]:+.3f}\n"
                    f"roll {r:+6.1f}   pitch {p:+6.1f}   yaw {yw:+6.1f}  (deg)",
                    position="upper_left", font_size=11, name="hud", color="white",
                )
            except Exception as exc:
                print("Render update failed:", repr(exc))
                break

        try:
            pl.update()  # process UI events + repaint
        except Exception:
            break  # window was closed
        time.sleep(period)

    stopEvent.set()


if __name__ == "__main__":
    main()
