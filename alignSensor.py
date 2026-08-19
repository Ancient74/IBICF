import serial
import threading
import queue
import itertools
import numpy as np
from matplotlib import pyplot as plt

PORT = "COM7"
BAUD_RATE = 115200

# Firmware prints two interleaved lines, both scaled x1000:
#   acc:    <gx> <gy> <gz>     (gravity, as the accelerometer reports it)
#   magnet: <mx> <my> <mz>     (field, ALREADY hard/soft-iron corrected)
ACC_PREFIX = "acc"
MAG_PREFIX = "magnet"

# World references, in the SAME convention your MEKF uses:
#   GRAVITY_DIR  -> the direction the normalized accel points at rest
#   REFERENCE_FIELD -> the Earth field vector you feed the filter
# These only set the target dip angle that the field makes with gravity.
GRAVITY_DIR     = np.array([0.0, 0.0, 1.0])
REFERENCE_FIELD = np.array([0.434, 0.061, 0.899])

# Cosine of the fixed angle between field and gravity. None -> derive it from
# the reference vectors above (recommended; you already have them).
TARGET_COS = None

MIN_FIT_POINTS = 300       # alignment needs broad orientation coverage
ACC_NORM_TOL = 0.20        # drop samples with dynamic acceleration
DISPLAY_CAP = 2000

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


def normalizeRows(P):
    n = np.linalg.norm(P, axis=1, keepdims=True)
    n[n == 0.0] = 1.0
    return P / n


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def so3Exp(w):
    """Rodrigues: rotation matrix from a rotation vector."""
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    K = skew(k)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def rightAngleRotations():
    """The 24 proper signed-permutation matrices (cube rotation group)."""
    mats = []
    eye = np.eye(3)
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1.0, -1.0), repeat=3):
            R = np.zeros((3, 3))
            for col, (p, s) in enumerate(zip(perm, signs)):
                R[:, col] = s * eye[:, p]
            if abs(np.linalg.det(R) - 1.0) < 1e-6:
                mats.append(R)
    return mats


def coarseAlign(g, m, targetCos):
    """Pick the best right-angle mounting as a starting point."""
    best, bestCost = np.eye(3), np.inf
    for R in rightAngleRotations():
        dots = np.sum(g * (m @ R.T), axis=1)
        cost = float(np.sum((dots - targetCos) ** 2))
        if cost < bestCost:
            best, bestCost = R, cost
    return best


def refineAlign(R, g, m, targetCos, iters=100, tol=1e-10):
    """Gauss-Newton on SO(3): make accel . (R*mag) equal targetCos everywhere.

    residual_i   = g_i . (R m_i) - targetCos
    d(res_i)/dδ  = (R m_i) x g_i        (perturbation R -> exp([δ]x) R)
    """
    for _ in range(iters):
        Rm = m @ R.T
        res = np.sum(g * Rm, axis=1) - targetCos
        J = np.cross(Rm, g)                      # (N, 3)
        JtJ = J.T @ J + 1e-9 * np.eye(3)
        try:
            delta = -np.linalg.solve(JtJ, J.T @ res)
        except np.linalg.LinAlgError:
            break
        R = so3Exp(delta) @ R
        if np.linalg.norm(delta) < tol:
            break

    # snap back onto SO(3) to clean float drift
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def solveAlignment(accPts, magPts, targetCos):
    """Returns (R_align, kept_g, kept_m) or None if degenerate."""
    acc = np.asarray(accPts, dtype=float)
    mag = np.asarray(magPts, dtype=float)

    # reject dynamic-acceleration samples (accel no longer ~ pure gravity)
    accN = np.linalg.norm(acc, axis=1)
    med = np.median(accN)
    keep = np.abs(accN - med) <= ACC_NORM_TOL * med
    acc, mag = acc[keep], mag[keep]
    if len(acc) < MIN_FIT_POINTS:
        return None

    g = normalizeRows(acc)
    m = normalizeRows(mag)

    R = coarseAlign(g, m, targetCos)
    R = refineAlign(R, g, m, targetCos)
    return R, g, m


def nearestPreset(R):
    """Closest right-angle mounting and the geodesic distance to it (deg)."""
    best, bestTr = np.eye(3), -np.inf
    for C in rightAngleRotations():
        tr = np.trace(C.T @ R)
        if tr > bestTr:
            best, bestTr = C, tr
    angle = np.degrees(np.arccos(np.clip((bestTr - 1.0) / 2.0, -1.0, 1.0)))
    return best, angle


def drawCoverage(ax, g, Rm, title):
    elev, azim = ax.elev, ax.azim
    ax.cla()
    ax.scatter(g[:, 0], g[:, 1], g[:, 2], s=4, alpha=0.4, label="gravity (accel)")
    ax.scatter(Rm[:, 0], Rm[:, 1], Rm[:, 2], s=4, alpha=0.4, label="R*field")
    ax.set_title(title)
    ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2); ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_box_aspect((1, 1, 1))
    ax.legend(loc="upper right", fontsize=8)
    ax.view_init(elev=elev, azim=azim)


def drawConsistency(ax, angles, targetAngle):
    ax.cla()
    ax.hist(angles, bins=40, alpha=0.7)
    ax.axvline(targetAngle, color="red", linewidth=1.0,
               label=f"target {targetAngle:.1f} deg")
    ax.set_title("field-vs-gravity angle (should collapse to one line)")
    ax.set_xlabel("degrees"); ax.set_ylabel("count")
    ax.legend(fontsize=8)


def calibrate(targetCos):
    accPts, magPts = [], []
    latestAcc = None

    plt.ion()
    fig = plt.figure(figsize=(13, 6))
    axCov = fig.add_subplot(1, 2, 1, projection="3d")
    axCon = fig.add_subplot(1, 2, 2)
    fig.suptitle("mag/IMU alignment - tumble through ALL orientations slowly, "
                 "then close the window")

    while plt.fignum_exists(fig.number) and not stopEvent.is_set():
        try:
            while True:
                prefix, vals = samples.get_nowait()
                if prefix == ACC_PREFIX:
                    latestAcc = vals
                elif prefix == MAG_PREFIX and latestAcc is not None:
                    accPts.append(latestAcc)
                    magPts.append(vals)
        except queue.Empty:
            pass

        if len(magPts) >= MIN_FIT_POINTS:
            result = solveAlignment(accPts, magPts, targetCos)
            if result is not None:
                R, g, m = result
                if len(g) > DISPLAY_CAP:
                    idx = np.linspace(0, len(g) - 1, DISPLAY_CAP).astype(int)
                    g, m = g[idx], m[idx]
                Rm = m @ R.T
                dots = np.clip(np.sum(g * Rm, axis=1), -1.0, 1.0)
                angles = np.degrees(np.arccos(dots))
                tgt = np.degrees(np.arccos(np.clip(targetCos, -1.0, 1.0)))
                drawCoverage(axCov, g, Rm,
                             f"orientation coverage  (n={len(magPts)})")
                drawConsistency(axCon, angles, tgt)

        plt.pause(0.05)

    stopEvent.set()
    return accPts, magPts


def reportAlignment(accPts, magPts, targetCos):
    print("\n=== mag -> IMU alignment ===")
    result = solveAlignment(accPts, magPts, targetCos)
    if result is None:
        print(f"only {len(magPts)} usable samples - not enough "
              f"(need >= {MIN_FIT_POINTS}, with broad coverage).")
        return

    R, g, m = result
    dots = np.clip(np.sum(g * (m @ R.T), axis=1), -1.0, 1.0)
    angles = np.degrees(np.arccos(dots))
    targetAngle = np.degrees(np.arccos(np.clip(targetCos, -1.0, 1.0)))
    rms = float(np.sqrt(np.mean((angles - targetAngle) ** 2)))

    preset, presetAngle = nearestPreset(R)

    print(f"samples used:      {len(g)}")
    print(f"target dip angle:  {targetAngle:.2f} deg")
    print(f"achieved mean:     {angles.mean():.2f} deg")
    print(f"fit residual RMS:  {rms:.2f} deg")
    print(f"nearest 90-deg mount is {presetAngle:.1f} deg away")
    if presetAngle < 6.0:
        print("  -> essentially a clean right-angle mount; the integer matrix")
        print("     below is more repeatable than the fitted one.")
    if rms > 5.0:
        print("  WARNING: large residual. Check that 'magnet' is already")
        print("  hard/soft-iron corrected, that you tumbled through enough")
        print("  orientations, and that GRAVITY_DIR matches your accel sign.")

    print("\n// paste into firmware  (apply AFTER hard/soft-iron correction)")
    print("constexpr float magAlign[3][3] = {")
    for row in R:
        print(f"    {{ {row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f }},")
    print("};")
    print("// body_mag = magAlign * mag_calibrated")

    if presetAngle < 6.0:
        print("\n// clean right-angle alternative")
        print("constexpr float magAlign[3][3] = {")
        for row in preset:
            print(f"    {{ {row[0]:.1f}f, {row[1]:.1f}f, {row[2]:.1f}f }},")
        print("};")


def main():
    targetCos = (TARGET_COS if TARGET_COS is not None
                 else float(np.dot(REFERENCE_FIELD / np.linalg.norm(REFERENCE_FIELD),
                                   GRAVITY_DIR / np.linalg.norm(GRAVITY_DIR))))

    worker = threading.Thread(target=readSerial, daemon=True)
    worker.start()
    accPts, magPts = calibrate(targetCos)
    reportAlignment(accPts, magPts, targetCos)


if __name__ == "__main__":
    main()
