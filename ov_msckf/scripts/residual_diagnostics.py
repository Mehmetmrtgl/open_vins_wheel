#!/usr/bin/env python3
"""Offline diagnostics for the platform update residual log.

Consumes the CSV written by UpdaterPlatform when platform.log_path
is set. Produces, for the lateral (y) residual:

  1. Lag-k autocorrelation of the residual sequence, plus the decorrelation
     time (first lag where the autocorrelation drops below 1/e). Tests whether
     the residual error is white, which the per-sample EKF update assumes.
  2. Regression of the lateral residual on angular velocity z. The slope
     magnitude estimates the longitudinal offset, in meters, between the
     configured platform origin and the true point where lateral velocity
     vanishes (the rear axle for an Ackermann vehicle).
  3. Regression of the lateral residual on angular velocity x. A slope
     magnitude near the configured vertical lever arm (1.7 m) means the rigid
     lever-arm term injects roll-rate error that the real vehicle (suspension)
     does not perform; a slope near zero means the lever-arm term is tracking
     real motion.
  4. Signed mean of the residual over 30-second windows, against the band
     +/- 3*sigma_hat/sqrt(N) expected if the residual were white and unbiased.

Usage:
    python3 residual_diagnostics.py /path/to/constraint_log.csv

Only numpy is required.
"""

import sys

import numpy as np

LEVER_ARM_Z = 1.7          # configured |p_OinI| vertical component, meters
WINDOW_SEC = 30.0          # windowed-mean length
MAX_LAG = 300              # autocorrelation horizon in samples
LATERAL_AXIS = 1           # body y


def autocorr(x, max_lag):
    x = x - x.mean()
    var = np.dot(x, x) / len(x)
    if var <= 0.0:
        return np.zeros(max_lag + 1)
    r = np.empty(max_lag + 1)
    for k in range(max_lag + 1):
        r[k] = np.dot(x[: len(x) - k], x[k:]) / ((len(x) - k) * var)
    return r


def regress(y, x):
    """Least squares y = a*x + b. Returns slope, intercept, Pearson r."""
    A = np.vstack([x, np.ones_like(x)]).T
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    denom = x.std() * y.std()
    r = float(np.corrcoef(x, y)[0, 1]) if denom > 0 else 0.0
    return float(a), float(b), r


def main(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t"]
    n = len(t)
    if n < 100:
        sys.exit(f"only {n} rows in {path}, need a full run")
    dt = np.median(np.diff(t))
    print(f"rows: {n}   span: {t[-1] - t[0]:.1f} s   median dt: {dt:.4f} s "
          f"({1.0 / dt:.1f} Hz)   accepted: {100.0 * data['applied'].mean():.1f} %")

    # Map the lateral residual out of the (axis0, axis1) column pairs.
    res_y = np.where(data["axis0"] == LATERAL_AXIS, data["res0"],
                     np.where(data["axis1"] == LATERAL_AXIS, data["res1"], np.nan))
    ok = np.isfinite(res_y)
    res_y, t, wx, wz = res_y[ok], t[ok], data["whx"][ok], data["whz"][ok]
    print(f"lateral residual: mean {res_y.mean():+.5f} m/s   "
          f"std {res_y.std():.5f} m/s   (global signed mean, the finding-4 statistic)")

    # 1. Whiteness.
    max_lag = min(MAX_LAG, len(res_y) // 4)
    r = autocorr(res_y, max_lag)
    below = np.nonzero(r[1:] < 1.0 / np.e)[0]
    tau = (below[0] + 1) * dt if len(below) else float("inf")
    print("\n[1] autocorrelation of the lateral residual")
    for k in (1, 2, 5, 10, 20, 50, 100):
        if k <= max_lag:
            print(f"    lag {k:4d} ({k * dt:6.2f} s): r = {r[k]:+.3f}")
    print(f"    decorrelation time (r < 1/e): {tau:.2f} s"
          f"   -> principled update_min_dt >= {tau:.1f}")
    print("    verdict: " + ("RESIDUAL IS TIME-CORRELATED, per-sample white-noise "
                             "updates over-count it"
                             if r[1] >= 0.2 else
                             "residual is close to white; the over-counting "
                             "mechanism is NOT supported at this rate"))

    # 2. Longitudinal offset from yaw rate.
    a_z, b_z, r_z = regress(res_y, wz)
    print("\n[2] lateral residual vs angular velocity z (yaw rate)")
    print(f"    slope {a_z:+.4f} m   intercept {b_z:+.5f} m/s   corr {r_z:+.3f}")
    print(f"    implied longitudinal offset |L| = {abs(a_z):.3f} m")
    print("    verdict: " + ("offset between configured origin and true "
                             "zero-lateral-velocity point is REAL"
                             if abs(a_z) >= 0.3 and abs(r_z) >= 0.3 else
                             ("offset is negligible; the reference-point "
                              "inference is FALSIFIED" if abs(a_z) < 0.1 else
                              "inconclusive, weak correlation")))

    # 3. Lever-arm roll leakage.
    a_x, b_x, r_x = regress(res_y, wx)
    print("\n[3] lateral residual vs angular velocity x (roll rate)")
    print(f"    slope {a_x:+.4f} m   intercept {b_x:+.5f} m/s   corr {r_x:+.3f}")
    print(f"    configured vertical lever arm: {LEVER_ARM_Z:.2f} m")
    print("    verdict: " + ("rigid lever-arm term injects roll-rate error "
                             "(suspension decouples body roll from contact "
                             "point): leakage is REAL"
                             if abs(a_x) >= 0.5 * LEVER_ARM_Z and abs(r_x) >= 0.3 else
                             ("lever-arm term tracks real motion; the leakage "
                              "inference is FALSIFIED" if abs(a_x) < 0.3 else
                              "inconclusive, weak correlation")))

    # 4. Windowed signed means.
    print(f"\n[4] signed mean over {WINDOW_SEC:.0f} s windows")
    edges = np.arange(t[0], t[-1], WINDOW_SEC)
    sigma = res_y.std()
    outside = 0
    total = 0
    for lo in edges:
        sel = (t >= lo) & (t < lo + WINDOW_SEC)
        nw = int(sel.sum())
        if nw < 10:
            continue
        total += 1
        band = 3.0 * sigma / np.sqrt(nw)
        m = res_y[sel].mean()
        if abs(m) > band:
            outside += 1
    frac = 100.0 * outside / max(total, 1)
    print(f"    windows: {total}   outside the +/-3 sigma/sqrt(N) white-noise "
          f"band: {outside} ({frac:.1f} %)")
    print("    verdict: " + ("windowed bias far exceeds the white-noise band: "
                             "the residual carries low-frequency structure a "
                             "global signed mean cannot see"
                             if frac > 15.0 else
                             "windowed means consistent with an unbiased white "
                             "residual; the bias mechanism is NOT supported"))

    print("\njoint reading: if [1] shows correlation AND ([2] or [3] or [4]) "
          "shows structure, the constraint residual is biased and colored and "
          "the white-noise pseudo-measurement formulation over-counts it at "
          "any tight sigma. If [1] through [4] all come back clean, an "
          "appropriately weighted constraint could legitimately reduce "
          "variance, and the redundancy claim is the part to revisit.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: residual_diagnostics.py <constraint_log.csv>")
    main(sys.argv[1])
