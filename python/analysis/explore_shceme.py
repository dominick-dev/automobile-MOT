import h5py
import numpy as np
import matplotlib.pyplot as plt
import os, sys
from pathlib import Path

# grab path to data
REPO = Path(__file__).resolve().parents[2]
DATA = Path(os.environ.get("RT_DATA_DIR", REPO / "data/RadarScenes/data"))
seq = sys.argv[1] if len(sys.argv) > 1 else "sequence_1"
f = h5py.File(DATA / seq / "radar_data.h5", "r")

# initial h5 scheme lookup
print(list(f.keys()))
d1 = f[list(f.keys())[0]]
print(d1.dtype.names)
print(d1[0])

print("")

d2 = f[list(f.keys())[1]]
print(d2.dtype.names)
print(d2[0])

# look more into timestaps and detections
d = f["radar_data"]
t = d["timestamp"][:]
print(np.unique(d["sensor_id"][:]))          # confirm 4 sensors
print(len(np.unique(t)))                      # number of distinct timestamps
print(np.bincount(np.unique(t, return_counts=True)[1]))  # detections per timestamp

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

# plot sequence, get idea of what detections look like for a given sequence
t0 = ut[2000]
mask = (t >= t0) & (t < t0 + 75_000)     # 75 ms window, all four sensors
x, y = d["x_cc"][mask], d["y_cc"][mask]
lbl = d["label_id"][mask]
plt.scatter(x, y, c=lbl, s=4, cmap="tab20")
plt.gca().set_aspect("equal")
plt.xlabel("x_cc (m, forward)"); plt.ylabel("y_cc (m, left)")
plt.show()
