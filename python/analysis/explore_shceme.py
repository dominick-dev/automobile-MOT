import h5py
import numpy as np
import matplotlib.pyplot as plt
import os, sys
from pathlib import Path

import json

# grab path to data
REPO = Path(__file__).resolve().parents[2]
DATA = Path(os.environ.get("RT_DATA_DIR", REPO / "data/RadarScenes/data"))
seq = sys.argv[1] if len(sys.argv) > 1 else "sequence_1"
with h5py.File(DATA / seq / "radar_data.h5", "r") as f:

# initial h5 scheme lookup
    print(list(f.keys()))
    d1 = f[list(f.keys())[0]]
    print(d1.dtype.names)
    print(d1[0])

    print()

    d2 = f[list(f.keys())[1]]
    print(d2.dtype.names)
    print(d2[0])

# look more into timestaps and detections
    d = f["radar_data"]
    t = d["timestamp"][:]
    print(np.unique(d["sensor_id"][:]))          # confirm 4 sensors
    print(len(np.unique(t)))                      # number of distinct timestamps
    print(np.bincount(np.unique(t, return_counts=True)[1]))  # detections per timestamp
    print()
    print(d.dtype)

# sequence specific info
    t0 = np.unique(t)[500]                    # any timestamp, not the first
    print(np.unique(d["sensor_id"][t == t0])) # one value, or four?
    print((t.max() - t.min()) / 1e6, "seconds")
    print(len(d), "total detections")

# get idea of sensor collect routine
    ut = np.unique(t)
    print(np.diff(ut)[:20] / 1e3, "ms between consecutive timestamps")
    s1 = np.unique(t[d["sensor_id"][:] == 1])
    print(np.diff(s1)[:20] / 1e3, "ms between sensor 1 measurements")

# odometry table info
    o = f["odometry"]
    ot = o["timestamp"][:]
    print(len(ot), "odometry rows")
    print(np.diff(ot)[:20] / 1e3, "ms between odometry samples")
    print()
    print(o.dtype)

# plot sequence, get idea of what detections look like for a given sequence
    t0 = ut[2000]
    mask = (t >= t0) & (t < t0 + 75_000)     # 75 ms window, all four sensors
    x, y = d["x_cc"][mask], d["y_cc"][mask]
    lbl = d["label_id"][mask]
    plt.scatter(x, y, c=lbl, s=4, cmap="tab20")
    plt.gca().set_aspect("equal")
    plt.xlabel("x_cc (m, forward)"); plt.ylabel("y_cc (m, left)")
    plt.show()

    print(d.dtype.itemsize, "bytes per row")
    for n in d.dtype.names:
        print(f"{n:16} {str(d.dtype[n]):8} offset {d.dtype.fields[n][1]}")


    odo = f["odometry"][:]
    all_det = f["radar_data"][:]

    # seeded random sample across the whole recording, not just its start.
    rng = np.random.default_rng(0)
    det = all_det[np.sort(rng.choice(len(all_det), 20_000, replace=False))]

    t_odo = odo["timestamp"]
    t_det = det["timestamp"]
    t0 = t_odo[0]

    # bracketing odometry samples for each detection: t_odo[lo] <= t_det <= t_odo[hi].
    # clipping hi to >= 1 keeps lo valid; points outside the odometry range clamp to the ends.
    hi = np.clip(np.searchsorted(t_odo, t_det), 1, len(odo) - 1)
    lo = hi - 1
    nearest = np.where(t_det - t_odo[lo] < t_odo[hi] - t_det, lo, hi)


def to_seq(px, py, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    xs = px + c * det["x_cc"] - s * det["y_cc"]
    ys = py + s * det["x_cc"] + c * det["y_cc"]
    return xs, ys


def report(name, xs, ys):
    err = np.hypot(xs - det["x_seq"], ys - det["y_seq"])
    p50, p99 = np.percentile(err, [50, 99])
    print(f"{name:13} median {p50:.3f} m   p99 {p99:.3f} m   max {err.max():.3f} m")
    return err


# Transform check using the nearest odometry sample.
err_nearest = report(
    "nearest",
    *to_seq(odo["x_seq"][nearest], odo["y_seq"][nearest], odo["yaw_seq"][nearest]),
)

# Same check with pose linearly interpolated to each detection's time.
#    Yaw is interpolated the short way round, so a wrap at +-pi doesn't spin 360 degrees.
alpha = np.clip((t_det - t_odo[lo]) / (t_odo[hi] - t_odo[lo]), 0.0, 1.0)
dyaw = odo["yaw_seq"][hi] - odo["yaw_seq"][lo]
dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))

px = odo["x_seq"][lo] + alpha * (odo["x_seq"][hi] - odo["x_seq"][lo])
py = odo["y_seq"][lo] + alpha * (odo["y_seq"][hi] - odo["y_seq"][lo])
pyaw = odo["yaw_seq"][lo] + alpha * dyaw
err_interp = report("interpolated", *to_seq(px, py, pyaw))

# Look at the worst points rather than discounting them.
print("\nworst 5 (interpolated):")
for k in np.argsort(err_interp)[::-1][:5]:
    print(
        f"  t={(t_det[k] - t0) / 1e6:7.3f} s  sensor {det['sensor_id'][k]}  "
        f"err {err_interp[k]:.3f} m  range {det['range_sc'][k]:5.1f} m  "
        f"ego vx {odo['vx'][nearest[k]]:4.1f} m/s  yaw rate {odo['yaw_rate'][nearest[k]]:+.3f} rad/s"
    )

# Radial velocity sign, using only points where the ego is actually moving.
#    Driving forward, stationary objects ahead should close (vr < 0)
#    and stationary objects behind should recede (vr > 0).
moving = odo["vx"][nearest] > 2.0
stationary = np.abs(det["vr_compensated"]) < 0.1
print(f"\nego moving in {moving.mean():.0%} of sampled detections")

checks = [
    ("ahead", det["x_cc"] > 5.0, det["vr"] < 0, "vr < 0"),
    ("behind", det["x_cc"] < -5.0, det["vr"] > 0, "vr > 0"),
]
for label, region, expected, desc in checks:
    mask = moving & stationary & region
    if mask.sum() == 0:
        print(f"{label:6}: no qualifying points")
        continue
    print(f"{label:6}: {mask.sum():5d} points, fraction with {desc}: {expected[mask].mean():.3f}")


print("--- entering new---")
with open(Path(os.environ.get("RT_DATA_DIR", REPO / "data/RadarScenes/data")) / "sensors.json") as sf:
    sensors_raw = json.load(sf)

# index by sensor_id (1..4) instead of the "radar_N" key
sensors = {v["id"]: v for v in sensors_raw.values()}
for sid, s in sorted(sensors.items()):
    print(sid, s["x"], s["y"], s["yaw"])

sx = np.array([sensors[s]["x"] for s in det["sensor_id"]])
sy = np.array([sensors[s]["y"] for s in det["sensor_id"]])
syaw = np.array([sensors[s]["yaw"] for s in det["sensor_id"]])

# sensor-frame polar -> sensor-frame Cartesian -> car frame
lx = det["range_sc"] * np.cos(det["azimuth_sc"])
ly = det["range_sc"] * np.sin(det["azimuth_sc"])
c, s = np.cos(syaw), np.sin(syaw)
cx = sx + c * lx - s * ly
cy = sy + s * lx + c * ly

err = np.hypot(cx - det["x_cc"], cy - det["y_cc"])
p50, p99 = np.percentile(err, [50, 99])
print(f"mounting check: median {p50:.4f} m   p99 {p99:.4f} m   max {err.max():.4f} m")

# ego linear + rotational velocity at the sensor mount point (car frame, at detection time)
vx_ego = odo["vx"][nearest]
wz = odo["yaw_rate"][nearest]
sensor_vx = vx_ego - wz * sy          # rigid-body velocity at (sx, sy)
sensor_vy = wz * sx

# unit line-of-sight vector, in the car frame, from the sensor to the point
az_car = det["azimuth_sc"] + syaw     # sensor-frame azimuth rotated into car frame
ux, uy = np.cos(az_car), np.sin(az_car)

# component of ego motion along that line of sight, removed from the raw measurement
ego_radial = sensor_vx * ux + sensor_vy * uy
vr_comp_mine = det["vr"] + ego_radial

err = np.abs(vr_comp_mine - det["vr_compensated"])
p50, p99 = np.percentile(err, [50, 99])
print(f"vr compensation check: median {p50:.4f} m/s   p99 {p99:.4f} m/s   max {err.max():.4f} m/s")
