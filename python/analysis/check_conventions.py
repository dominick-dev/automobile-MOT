"""Verify RadarScenes conventions across every sequence.

Per sequence, checks:
  - car frame -> sequence frame transform, with interpolated ego pose
  - sensor mounting: sensor-frame polar -> car frame
  - ego-motion compensation against the dataset's vr_compensated
  - radial velocity sign and compensation, using ground-truth static labels
  - per-sensor timing: interval spread and 75 ms window occupancy
  - odometry coverage: gaps at each end and frames dropped by the boundary rule
  - non-finite values in raw and pose-dependent fields

Prints one row per sequence, a pooled summary, and any sequence that fails.

Usage:
  python check_conventions.py                  # all sequences
  python check_conventions.py sequence_1 ...   # specific ones
"""

import json
import os
import sys
from pathlib import Path

import h5py
import numpy as np

REPO = Path(__file__).resolve().parents[2]
DATA = Path(os.environ.get("RT_DATA_DIR", REPO / "data/RadarScenes/data"))

SAMPLE_SIZE = 20_000       # detections sampled per sequence for the geometric checks
WINDOW_US = 75_000         # frame window, microseconds
MOVING_MPS = 2.0           # a sequence whose ego never exceeds this is stationary-ego
MIN_EGO_RADIAL_MPS = 1.0   # ego line-of-sight speed needed for an unambiguous sign test
LONG_GAP_MS = 500.0        # per-sensor gaps longer than this are listed individually
STATIC_LABEL = 11          # RadarScenes label_id for static environment

# Failure thresholds. A sequence exceeding any of these is listed at the end.
MAX_POS_P99_M = 0.05
MAX_MOUNT_M = 1e-3
MAX_VR_P99_MPS = 0.1
MIN_SIGN_FRAC = 0.97

RAW_FIELDS = ("range_sc", "azimuth_sc", "rcs", "vr", "x_cc", "y_cc")
ORACLE_FIELDS = ("x_seq", "y_seq", "vr_compensated")


def load_mounting():
    """Return sensor ids and mounting x, y, yaw as arrays indexed by sensor_id."""
    with open(DATA / "sensors.json") as f:
        raw = json.load(f)
    by_id = {v["id"]: v for v in raw.values()}
    size = max(by_id) + 1
    mx, my, myaw = np.zeros(size), np.zeros(size), np.zeros(size)
    for i, s in by_id.items():
        mx[i], my[i], myaw[i] = s["x"], s["y"], s["yaw"]
    return sorted(by_id), mx, my, myaw


def sequence_dirs(names):
    """Sequences named on the command line, or every sequence, in numeric order."""
    if names:
        return [DATA / n for n in names]
    dirs = [p for p in DATA.glob("sequence_*") if (p / "radar_data.h5").exists()]
    return sorted(dirs, key=lambda p: int(p.name.split("_")[1]))


def check_sequence(seq_dir, mounting):
    ids, mx, my, myaw = mounting
    seq_number = int(seq_dir.name.split("_")[1])

    with h5py.File(seq_dir / "radar_data.h5", "r") as f:
        odo = f["odometry"][:]
        all_det = f["radar_data"][:]

    t_odo = odo["timestamp"]
    t_all = all_det["timestamp"]
    sid_all = all_det["sensor_id"]
    t_first, t_last = t_all.min(), t_all.max()

    r = {
        "name": seq_dir.name,
        "odo_sorted": bool(np.all(np.diff(t_odo) > 0)),
        "vmax": float(np.max(odo["vx"])),
        "gap_before_ms": max(0.0, (t_odo[0] - t_first) / 1e3),
        "gap_after_ms": max(0.0, (t_last - t_odo[-1]) / 1e3),
    }

    # Frames dropped by the boundary rule: a frame is kept only if its whole
    # window [start, start + W) lies within odometry coverage.
    n_windows = int((t_last - t_first) // WINDOW_US) + 1
    starts = t_first + np.arange(n_windows, dtype=np.int64) * WINDOW_US
    kept = (starts >= t_odo[0]) & (starts + WINDOW_US - 1 <= t_odo[-1])
    r["dropped"] = int((~kept).sum())

    # Non-finite values. Raw fields should never have any; pose-dependent oracle
    # fields are expected to be undefined only outside odometry coverage.
    in_cov = (t_all >= t_odo[0]) & (t_all <= t_odo[-1])
    oracle_ok = np.ones(len(all_det), dtype=bool)
    for field in ORACLE_FIELDS:
        oracle_ok &= np.isfinite(all_det[field])
    r["raw_nan"] = int(sum((~np.isfinite(all_det[field])).sum() for field in RAW_FIELDS))
    r["nan_rows"] = int((~oracle_ok).sum())
    r["nan_in_cov"] = int((~oracle_ok & in_cov).sum())

    # Per-sensor timing, on the full data rather than the sample.
    worst_ms, worst_sensor, worst_t = 0.0, None, None
    zero = two_plus = total = 0
    for s in ids:
        ts = np.unique(t_all[sid_all == s])
        if len(ts) < 2:
            continue
        dt = np.diff(ts)
        k = int(np.argmax(dt))
        if dt[k] / 1e3 > worst_ms:
            worst_ms, worst_sensor, worst_t = dt[k] / 1e3, s, (ts[k] - t_first) / 1e6
        counts = np.bincount((ts - t_first) // WINDOW_US, minlength=n_windows)
        zero += int((counts == 0).sum())
        two_plus += int((counts >= 2).sum())
        total += counts.size
    r.update(
        max_dt=worst_ms,
        max_dt_sensor=worst_sensor,
        max_dt_t=worst_t,
        zero_pct=100 * zero / total if total else np.nan,
        two_pct=100 * two_plus / total if total else np.nan,
    )

    # Sample only detections inside odometry coverage with defined oracle values.
    # Seeded per sequence, so checking one sequence alone gives the same sample.
    usable = np.flatnonzero(in_cov & oracle_ok)
    n = min(SAMPLE_SIZE, len(usable))
    if n == 0:
        r.update(pos_p50=np.nan, pos_p99=np.nan, mount_max=np.nan, vr_p99=np.nan,
                 sign=np.nan, sign_n=0, sign_hits=0, static_resid=np.nan)
        return r, np.array([]), np.array([])
    rng = np.random.default_rng(seq_number)
    det = all_det[np.sort(rng.choice(usable, n, replace=False))]
    t = det["timestamp"]
    sid = det["sensor_id"]

    # Bracketing odometry samples and interpolation weight in [0, 1].
    hi = np.clip(np.searchsorted(t_odo, t), 1, len(odo) - 1)
    lo = hi - 1
    alpha = (t - t_odo[lo]) / np.maximum(t_odo[hi] - t_odo[lo], 1)

    def lerp(field):
        a, b = odo[field][lo], odo[field][hi]
        return a + alpha * (b - a)

    dyaw = odo["yaw_seq"][hi] - odo["yaw_seq"][lo]
    yaw = odo["yaw_seq"][lo] + alpha * np.arctan2(np.sin(dyaw), np.cos(dyaw))

    # Car frame -> sequence frame, compared with the dataset's x_seq/y_seq.
    c, s = np.cos(yaw), np.sin(yaw)
    xs = lerp("x_seq") + c * det["x_cc"] - s * det["y_cc"]
    ys = lerp("y_seq") + s * det["x_cc"] + c * det["y_cc"]
    err_pos = np.hypot(xs - det["x_seq"], ys - det["y_seq"])
    r["pos_p50"], r["pos_p99"] = np.percentile(err_pos, [50, 99])

    # Mounting: sensor-frame polar -> car frame.
    az_car = det["azimuth_sc"] + myaw[sid]
    cx = mx[sid] + det["range_sc"] * np.cos(az_car)
    cy = my[sid] + det["range_sc"] * np.sin(az_car)
    r["mount_max"] = np.hypot(cx - det["x_cc"], cy - det["y_cc"]).max()

    # Ego compensation, with odometry interpolated to each detection's own time.
    vx, wz = lerp("vx"), lerp("yaw_rate")
    sensor_vx = vx - wz * my[sid]
    sensor_vy = wz * mx[sid]
    ego_radial = sensor_vx * np.cos(az_car) + sensor_vy * np.sin(az_car)
    vr_mine = det["vr"] + ego_radial
    err_vr = np.abs(vr_mine - det["vr_compensated"])
    r["vr_p99"] = np.percentile(err_vr, 99)

    # Radial velocity sign, independent of the dataset's own compensation: for
    # objects labeled static, raw vr should be the negative of the ego's
    # line-of-sight motion. Tested only where that motion is large enough for
    # its sign to be unambiguous.
    static = det["label_id"] == STATIC_LABEL
    testable = static & (np.abs(ego_radial) > MIN_EGO_RADIAL_MPS)
    hits = int((np.sign(det["vr"][testable]) == -np.sign(ego_radial[testable])).sum())
    r["sign_n"], r["sign_hits"] = int(testable.sum()), hits
    r["sign"] = hits / r["sign_n"] if r["sign_n"] else np.nan

    # Independent compensation check: our compensated velocity for static
    # objects should be near zero, with no reference to vr_compensated.
    r["static_resid"] = float(np.median(np.abs(vr_mine[static]))) if static.any() else np.nan

    return r, err_pos, err_vr


def failures(r):
    reasons = []
    if not r["odo_sorted"]:
        reasons.append("odometry not strictly increasing")
    if r["raw_nan"]:
        reasons.append(f"{r['raw_nan']} non-finite raw values")
    if r["nan_in_cov"]:
        reasons.append(f"{r['nan_in_cov']} undefined oracle rows inside odometry coverage")
    if r["pos_p99"] > MAX_POS_P99_M:
        reasons.append(f"position p99 {r['pos_p99']:.3f} m")
    if r["mount_max"] > MAX_MOUNT_M:
        reasons.append(f"mounting max {r['mount_max']:.4f} m")
    if r["vr_p99"] > MAX_VR_P99_MPS:
        reasons.append(f"vr p99 {r['vr_p99']:.3f} m/s")
    if not np.isnan(r["sign"]) and r["sign"] < MIN_SIGN_FRAC:
        reasons.append(f"vr sign {r['sign']:.3f}")
    return reasons


def main():
    mounting = load_mounting()
    dirs = sequence_dirs(sys.argv[1:])
    print(f"checking {len(dirs)} sequence(s) in {DATA}\n")

    header = (f"{'sequence':>13} {'pos50':>6} {'pos99':>6} {'mount':>6} {'vr99':>6} "
              f"{'sign':>6} {'static':>6} {'vmax':>5} {'maxdt':>7} {'0-win':>6} "
              f"{'2-win':>6} {'gap<':>6} {'gap>':>6} {'drop':>4} {'nan':>5} {'nanin':>5}")
    units = (f"{'':>13} {'mm':>6} {'mm':>6} {'mm':>6} {'m/s':>6} {'frac':>6} {'m/s':>6} "
             f"{'m/s':>5} {'ms':>7} {'%':>6} {'%':>6} {'ms':>6} {'ms':>6} {'frm':>4} "
             f"{'rows':>5} {'rows':>5}")
    print(header)
    print(units)

    rows, all_pos, all_vr = [], [], []
    for d in dirs:
        r, err_pos, err_vr = check_sequence(d, mounting)
        rows.append(r)
        all_pos.append(err_pos)
        all_vr.append(err_vr)
        print(f"{r['name']:>13} {1e3 * r['pos_p50']:6.1f} {1e3 * r['pos_p99']:6.1f} "
              f"{1e3 * r['mount_max']:6.3f} {r['vr_p99']:6.3f} {r['sign']:6.3f} "
              f"{r['static_resid']:6.3f} {r['vmax']:5.1f} {r['max_dt']:7.1f} "
              f"{r['zero_pct']:6.2f} {r['two_pct']:6.2f} {r['gap_before_ms']:6.1f} "
              f"{r['gap_after_ms']:6.1f} {r['dropped']:4d} {r['nan_rows']:5d} "
              f"{r['nan_in_cov']:5d}", flush=True)

    all_pos = np.concatenate(all_pos)
    all_vr = np.concatenate(all_vr)
    sign_n = sum(r["sign_n"] for r in rows)
    sign_hits = sum(r["sign_hits"] for r in rows)

    print("\npooled across all sequences:")
    print(f"  position error      median {1e3 * np.median(all_pos):.1f} mm   "
          f"p99 {1e3 * np.percentile(all_pos, 99):.1f} mm")
    print(f"  vr error            median {np.median(all_vr):.4f} m/s   "
          f"p99 {np.percentile(all_vr, 99):.4f} m/s")
    if sign_n:
        print(f"  vr sign (static)    {sign_hits / sign_n:.4f} of {sign_n} test points")
    print(f"  static residual     median {np.nanmedian([r['static_resid'] for r in rows]):.4f} m/s")

    stationary = [r["name"] for r in rows if r["vmax"] < MOVING_MPS]
    print(f"\n  stationary-ego sequences (max speed < {MOVING_MPS} m/s): {len(stationary)}")

    print(f"  max per-sensor interval: {max(r['max_dt'] for r in rows):.1f} ms")
    for r in rows:
        if r["max_dt"] > LONG_GAP_MS:
            print(f"    {r['name']}: sensor {r['max_dt_sensor']} silent for "
                  f"{r['max_dt'] / 1e3:.2f} s from t={r['max_dt_t']:.1f} s")
    print(f"  windows missing a sensor {np.nanmean([r['zero_pct'] for r in rows]):.2f} %   "
          f"with a duplicate {np.nanmean([r['two_pct'] for r in rows]):.2f} %")

    before = [r["gap_before_ms"] for r in rows]
    after = [r["gap_after_ms"] for r in rows]
    dropped = [r["dropped"] for r in rows]
    print(f"\n  radar before odometry starts: {sum(b > 0 for b in before)} sequence(s), "
          f"up to {max(before):.1f} ms")
    print(f"  radar after odometry ends:    {sum(a > 0 for a in after)} sequence(s), "
          f"{min(after):.1f}-{max(after):.1f} ms")
    print(f"  frames dropped per sequence:  {min(dropped)}-{max(dropped)}")

    nan_seqs = [r["name"] for r in rows if r["nan_rows"]]
    print(f"\n  sequences with undefined oracle values: {len(nan_seqs)}"
          + (f" ({', '.join(nan_seqs)})" if nan_seqs else ""))
    print(f"  undefined oracle rows inside odometry coverage: "
          f"{sum(r['nan_in_cov'] for r in rows)}")
    print(f"  non-finite raw values: {sum(r['raw_nan'] for r in rows)}")

    failed = [(r["name"], failures(r)) for r in rows if failures(r)]
    print(f"\n{len(failed)} sequence(s) outside thresholds")
    for name, reasons in failed:
        print(f"  {name}: {'; '.join(reasons)}")


if __name__ == "__main__":
    main()
