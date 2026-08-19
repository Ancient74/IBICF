import serial
import threading
import queue
import numpy as np
from matplotlib import pyplot as plt

PORT = "COM7"
BAUD_RATE = 115200

# Which serial prefix to calibrate. Everything else is generic — switch this
# to "magnet" to calibrate the magnetometer.
TARGET = "acc"

# "sphere"   -> bias only          (corrected = raw - bias)            [vector]
# "ellipsoid"-> bias + scale/skew  (corrected = A @ (raw - bias))      [matrix]
CALIB_MODE = "ellipsoid"

# Radius the corrected cloud is normalized to. None = auto (use the raw
# sphere-fit radius, so output keeps the same units, e.g. ~1000 milli-g).
TARGET_RADIUS = None

DISPLAY_CAP = 2000
MIN_FIT_POINTS = 100   # ellipsoid needs more coverage than a plain sphere

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
            if parsed is not None and parsed[0] == TARGET:
                samples.put(parsed[1])


def fitSphere(points):
    """Bias-only fit. Returns (center, radius)."""
    p = np.asarray(points, dtype=float)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    A = np.column_stack((2 * x, 2 * y, 2 * z, np.ones(len(p))))
    b = x**2 + y**2 + z**2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    center = sol[:3]
    radius = float(np.sqrt(max(sol[3] + center @ center, 0.0)))
    return center, radius


def fitEllipsoid(points, targetRadius):
    """General ellipsoid fit. Returns (A, bias) for corrected = A @ (raw-bias),
    which lands corrected points on a sphere of `targetRadius`."""
    p = np.asarray(points, dtype=float)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    # a x^2 + b y^2 + c z^2 + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z = 1
    D = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z))
    v, *_ = np.linalg.lstsq(D, np.ones(len(p)), rcond=None)
    a, b_, c, d, e, f, g, h, i = v
    A3 = np.array([[a, d, e], [d, b_, f], [e, f, c]])
    b3 = np.array([g, h, i])
    center = -np.linalg.solve(A3, b3)          # bias
    k = 1.0 - b3 @ center                       # u^T (A3/k) u = 1
    M = A3 / k
    w, V = np.linalg.eigh(M)                     # symmetric, expected PD
    W = V @ np.diag(np.sqrt(np.abs(w))) @ V.T    # sqrtm(M): maps to unit sphere
    A = targetRadius * W
    return A, center


def sphereSurface(center, radius, n=24):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    sx = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    sy = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    sz = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    return sx, sy, sz


def drawCloud(ax, pts, sphereCenter, radius, title):
    elev, azim = ax.elev, ax.azim
    ax.cla()
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=4, alpha=0.5)
    if radius > 0:
        sx, sy, sz = sphereSurface(sphereCenter, radius)
        ax.plot_wireframe(sx, sy, sz, color="gray", alpha=0.25, linewidth=0.5)
        lim = 1.4 * radius
        ax.set_xlim(sphereCenter[0] - lim, sphereCenter[0] + lim)
        ax.set_ylim(sphereCenter[1] - lim, sphereCenter[1] + lim)
        ax.set_zlim(sphereCenter[2] - lim, sphereCenter[2] + lim)
    ax.set_title(title)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)


def subsample(arr, cap):
    if len(arr) <= cap:
        return arr
    return arr[::len(arr) // cap + 1]


def solve(arr):
    """Run the configured fit. Returns a result dict, or None if degenerate."""
    center, sphereR = fitSphere(arr)
    target = TARGET_RADIUS if TARGET_RADIUS is not None else sphereR
    if CALIB_MODE == "sphere":
        return {"mode": "sphere", "bias": center, "radius": sphereR}
    try:
        A, bias = fitEllipsoid(arr, target)
    except np.linalg.LinAlgError:
        return None
    return {"mode": "ellipsoid", "bias": bias, "A": A, "radius": target}


def applyCorrection(pts, res):
    d = pts - res["bias"]
    if res["mode"] == "ellipsoid":
        return d @ res["A"].T
    return d


def calibrate():
    allPts = []
    res = None

    plt.ion()
    fig = plt.figure(figsize=(13, 6))
    axRaw = fig.add_subplot(1, 2, 1, projection="3d")
    axCal = fig.add_subplot(1, 2, 2, projection="3d")
    fig.suptitle(f"{TARGET} {CALIB_MODE} calibration — rotate through all "
                 f"orientations, then close the window")

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        try:
            while True:
                allPts.append(samples.get_nowait())
        except queue.Empty:
            pass

        if len(allPts) >= MIN_FIT_POINTS:
            arr = np.asarray(allPts)
            r = solve(arr)
            if r is not None:
                res = r
                shown = subsample(arr, DISPLAY_CAP)
                drawCloud(axRaw, shown, res["bias"], res["radius"],
                          f"raw + fitted shape  (n={len(allPts)})")
                drawCloud(axCal, applyCorrection(shown, res),
                          np.zeros(3), res["radius"],
                          "corrected (centered on origin)")

        plt.pause(0.05)

    stopEvent.set()
    return np.asarray(allPts), res


def reportCalibration(pts, res):
    n = len(pts)
    print(f"\n=== {TARGET} calibration ({CALIB_MODE}) ===")
    if res is None or n < MIN_FIT_POINTS:
        print(f"only {n} samples — not enough for a reliable fit "
              f"(need >= {MIN_FIT_POINTS}).")
        return

    corr = applyCorrection(pts, res)
    mags = np.linalg.norm(corr, axis=1)
    rms = float(np.sqrt(np.mean((mags - res["radius"]) ** 2)))
    b = res["bias"]

    print(f"samples:          {n}")
    print(f"bias (offset):    x={b[0]:.6f}  y={b[1]:.6f}  z={b[2]:.6f}")
    print(f"target radius:    {res['radius']:.6f}")
    print(f"fit residual RMS: {rms:.6f}  ({100 * rms / res['radius']:.2f}% of radius)")

    print(f"\n// paste into firmware")
    print(f"constexpr float {TARGET}Bias[3] = "
          f"{{ {b[0]:.6f}f, {b[1]:.6f}f, {b[2]:.6f}f }};")
    if res["mode"] == "ellipsoid":
        A = res["A"]
        print(f"constexpr float {TARGET}Matrix[3][3] = {{")
        for row in A:
            print(f"    {{ {row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f }},")
        print("};")
        print(f"// corrected = {TARGET}Matrix * (raw - {TARGET}Bias)")
    else:
        print(f"// corrected = raw - {TARGET}Bias")


def main():
    worker = threading.Thread(target=readSerial, daemon=True)
    worker.start()
    pts, res = calibrate()
    reportCalibration(pts, res)


if __name__ == "__main__":
    main()
