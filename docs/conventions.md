# Coordinate, Unit, and Timing Conventions

This document is the contract every stage of the pipeline is written against.
Each convention was checked against the dataset's own fields on all 155
RadarScenes sequences available locally, using
`python/analysis/check_conventions.py`. Rerun that script when adding a
dataset or changing anything below.

This file supersedes earlier informal project notes. Where they disagree
(timestamps in milliseconds, round-robin frame assembly, tracking in the car
frame), this file is correct.

## Quick reference

| Quantity        | Convention                                                        |
|-----------------|-------------------------------------------------------------------|
| Car frame       | Right-handed, x forward, y left, meters, origin at ego reference point |
| Sequence frame  | Fixed to the world, arbitrary origin. Holds all tracking state    |
| Angles          | Radians, counterclockwise-positive, zero along +x                 |
| Radial velocity | m/s, negative = approaching                                       |
| Time            | `int64` µs inside the loader; `double` seconds from the first radar timestamp everywhere else |
| Ego pose        | Linearly interpolated at each detection's own timestamp           |
| Frames          | Fixed 75 ms bins from t = 0                                       |
| Pose boundary   | A frame is processed only if its whole window has odometry coverage |

## Two frames, two jobs

- **Car frame**: attached to the ego vehicle; moves and rotates with it.
  Used for raw detection geometry (sensor mounting is defined here) and for
  output ("where is that object relative to me, now").
- **Sequence frame**: attached to the ground; never moves. Used for all
  tracking state: clustering, prediction, association, and the Kalman filter.

Tracking cannot happen in the car frame. The car frame accelerates and
rotates whenever the ego does, so an object moving at constant velocity in
the world does not move at constant velocity in the car frame whenever the ego
brakes or turns. A constant-velocity motion model is only valid in a frame
where constant velocity means constant velocity, which is the sequence frame.

This remains causal and realistic: it requires only the ego's own odometry,
which a real vehicle has in real time. No ground truth or future information
is involved.

```
detection (sensor frame: range, azimuth, own timestamp)
  -> car frame       fixed rigid transform from sensor mounting
  -> sequence frame  ego pose interpolated at the detection's own timestamp
  -> clustering and tracking, in the sequence frame
  -> car frame       current ego pose, for output only
```

The pipeline computes sequence-frame positions itself from car-frame
positions and odometry. The dataset's `x_seq`, `y_seq`, and `vr_compensated`
fields are used only as test oracles, never as pipeline inputs. This keeps the
pipeline portable to datasets that don't provide them, and avoids values the
dataset leaves undefined (see "Dataset anomalies").

## Car frame

- **+x** forward, **+y** left, meters.
- **Origin**: the ego vehicle's reference point. The sensors are mounted
  3.66–3.86 m ahead of it, consistent with a rear-axle origin; this is
  inferred from the mounting table, not verified directly.
- Ego-motion formulas below assume no sideways velocity at the origin
  (no sideslip), a good approximation at the rear axle.

```
               +x (forward)
                ^
                |
   +y <---------o  ego reference point
   (left)
```

**Verification**: car-frame positions, transformed by interpolated ego pose,
reproduce the dataset's `x_seq`/`y_seq` to 1.2 mm median and 11.5 mm p99,
pooled across all sequences (per-sequence p99 between 0.3 and 19 mm). An error
in handedness or yaw sign would produce errors of meters.

## Sequence frame

RadarScenes' fixed world frame (`x_seq`, `y_seq`, `yaw_seq`). The origin is
arbitrary and is not the ego's starting point: `sequence_1` begins with the
ego at (-142.77, -213.88). Coordinates reach hundreds of meters, so pipeline
code stores them as `double`.

Track velocity is the derivative of sequence-frame position: the object's
over-ground velocity, with a single unambiguous meaning. (In the car frame,
"velocity" could mean the derivative of car-frame coordinates, the object's
velocity expressed in rotating car axes, or velocity relative to the ego.
These differ whenever the ego turns.)

**Output** transforms each confirmed track's position into the car frame
using the current frame's ego pose. The velocity reported alongside it is an
open decision; see the end of this file.

## Sensor mounting

From `sensors.json` at the dataset root, keyed by `sensor_id`. The same file
applies to every sequence.

| sensor_id | x (m) | y (m)  | yaw (rad) |
|-----------|-------|--------|-----------|
| 1         | 3.663 | -0.873 | -1.484    |
| 2         | 3.860 | -0.700 | -0.436    |
| 3         | 3.860 |  0.700 |  0.436    |
| 4         | 3.663 |  0.873 |  1.484    |

Detections arrive in sensor-frame polar coordinates. `azimuth_sc` is
counterclockwise-positive, like every other angle:

```
az_car = azimuth_sc + sensor_yaw
x_car  = sensor_x + range_sc * cos(az_car)
y_car  = sensor_y + range_sc * sin(az_car)
```

**Verification**: reproduces the dataset's `x_cc`/`y_cc` to under 0.01 mm in
every sequence.

## Radial velocity and ego-motion compensation

`vr` is a radial velocity: the rate of change of range along the line of
sight, not a 2D vector. **Negative means approaching.**

Compensation removes the ego's own motion along the line of sight, leaving
the object's over-ground velocity along that line (about 0 for stationary
objects). With `vx` and `yaw_rate` interpolated from odometry at the
detection's own timestamp, the velocity of the sensor's mount point is:

```
sensor_vx = vx - yaw_rate * sensor_y
sensor_vy =      yaw_rate * sensor_x

vr_compensated = vr + sensor_vx * cos(az_car) + sensor_vy * sin(az_car)
```

The pipeline implements this itself; the dataset's `vr_compensated` is its
test oracle.

**Verification**, two independent ways:

- Against the dataset's `vr_compensated`: 0.0006 m/s median and 0.013 m/s p99
  pooled, at most 0.026 m/s p99 in any sequence, consistent with `float32`
  rounding.
- Against ground-truth labels, without using `vr_compensated` at all: for
  points labeled static (`label_id` 11), the formula leaves a median residual
  of 0.045 m/s. A reversed sign convention would leave residuals of meters per
  second.

Where the ego's line-of-sight speed exceeds 1 m/s, 98% of static-labeled
points have raw `vr` of the expected sign. The remaining 2% are points labeled
static whose Doppler indicates motion above 1 m/s, most likely mislabels or
multipath reflections. Consequence for clutter filtering: a Doppler-based
static/moving split will misclassify about 2% of static returns as moving.

## Timing

- **Raw timestamps** in both tables are `int64` microseconds on an arbitrary
  per-sequence epoch. They stay integer inside the loader, so binning and
  comparison use exact integer arithmetic.
- **t = 0** is the earliest `radar_data` timestamp in the sequence. Odometry
  timestamps use the same offset. In every sequence odometry starts before
  the radar, so early odometry samples have negative times; they serve only
  as interpolation endpoints.
- **Everywhere downstream of the loader**, time is `double` seconds from
  t = 0. Double precision is ample: timestamps up to a few hundred seconds at
  microsecond resolution need about 9 significant digits of double's 15–16.
- **Rationale**: integer microseconds invite unit-mixing bugs at every `dt`
  computation. One unit everywhere keeps physics code consistent.

## Sensor timing

The four radars are not synchronized. Each `radar_data` timestamp holds one
measurement from one sensor. Each sensor free-runs at about 13.5 Hz (period
about 73.5 ms), about 54 Hz combined. Odometry at 100 Hz is therefore about
1.9x the combined radar rate, or 7.4x any single sensor.

Sensors miss scans. Across all sequences:

- Most sequences have a maximum per-sensor interval of 77–83 ms.
- Many contain intervals of about 145–160 ms or about 220 ms: one or two
  missed scans.
- Two sequences contain a long single-sensor dropout: `sequence_98`, sensor 2
  silent for 1.89 s from t = 85.2 s; `sequence_144`, sensor 3 silent for
  1.88 s from t = 74.5 s. About 25 frames each.

No stage may assume all four sensors contribute to a frame. A track in the
field of view of a silent sensor receives no detections and must coast rather
than be deleted.

## Frame construction

Frames are fixed, contiguous, half-open bins on the t = 0 timeline, so frame
boundaries are deterministic and identical on every run:

```
W = 75 ms
frame k covers        t in [k*W, (k+1)*W)
frame k's timestamp   (k + 0.5) * W
detection at t is in  frame floor(t / W)
```

Per-sensor occupancy is usually one measurement per frame, not always.
Averaged across all sequences, 0.66% of (frame, sensor) pairs have no
measurement and 1.32% have two. Neither is special-cased; clustering consumes
whatever a frame contains. Detections with identical timestamps fall in the
same frame and need no special handling.

A frame is processed only if its entire window lies within odometry coverage,
so every detection in it has bracketing odometry samples. Radar never starts
before odometry, but extends 105–173 ms past the end of odometry in every
sequence, so the last 2–4 frames of each sequence are dropped. Extrapolating
instead would produce wrong poses precisely where nothing can check them: in
`sequence_1`, the dataset's own values for its final scan disagree with
interpolation by about 1.7 m.

## Ego pose lookup

Ego pose at any timestamp is linearly interpolated between the two bracketing
odometry samples. Yaw is interpolated the short way around (via
`atan2(sin(Δyaw), cos(Δyaw))`), so a wrap near ±π does not produce a spurious
full rotation.

**Verification**: interpolation reproduces the dataset's `x_seq`/`y_seq` to
1.2 mm median. Nearest-sample lookup is worse in every moving sequence
(10–50 mm median, growing with ego speed), which indicates the dataset itself
was built from interpolated poses.

## Target-motion skew

Placing each detection with the ego pose at its own timestamp removes the
ego's motion exactly. It does not remove the **object's** own motion during
the frame window. Detections of one object, taken at different moments
within a frame, land where the object was at each of those moments.

The spread is the object's over-ground speed times the time span. Relative to
the frame timestamp, a detection can be up to 37.5 ms early or late; across a
full frame, up to 75 ms apart. For a car at 30 m/s, that is up to about 2.25 m
of spread within one frame. Clustering thresholds must tolerate it.

This is partly correctable. Each detection's `vr_compensated` is the object's
velocity along the line of sight, so each point can be moved along its line of
sight by `vr_compensated × (t_frame − t_detection)`. That removes the radial
part of the spread, which dominates for objects ahead or behind; only the
tangential part remains. Deferred until clustering is measured without it.

## Latency

Batching into 75 ms frames means a frame can be processed only after its
window closes: its earliest detection waits up to 75 ms. Interpolating ego
pose also requires the odometry sample after each detection, adding up to one
odometry interval (about 10 ms). Where within the window the frame is stamped
changes the reported time, not when the frame becomes available.

This is acceptable for the current offline, accuracy-scored goals. If onboard
latency becomes an optimization target, the options are shorter windows, or
processing each sensor scan as it arrives with asynchronous filter updates.

## Dataset anomalies

- **Undefined pose-dependent fields at sequence ends.** In sequences 149–157,
  the last 718–1061 detections have NaN `x_seq`, `y_seq`, and
  `vr_compensated`: the three fields that require an ego pose. In five of
  them (150, 151, 152, 154, 156), 84–180 of those rows fall just inside
  odometry coverage, so the dataset's valid-pose window ends one or two scans
  before its last odometry sample. Raw fields are finite in every sequence.
  The pipeline is unaffected because it computes these values itself; tests
  and evaluation must skip non-finite oracle values. All nine sequences are
  stationary-ego recordings.
- **Stationary-ego sequences.** 39 sequences never exceed 2 m/s of ego speed.
  They exercise tracking with a stationary ego but not ego-motion handling.
  Regression sets should include both kinds deliberately.
- **Label noise.** About 2% of static-labeled points have Doppler
  inconsistent with a static object (see "Radial velocity and ego-motion
  compensation"). Evaluation should expect it.
- **Missing sequences.** Sequences 5 and 68 are absent locally. Confirm
  against the dataset's published sequence count.

## Run mode vs. eval mode

RadarScenes' label fields (`track_id`, `uuid`, `label_id`) are ground truth,
produced offline with camera footage and future frames, so a causal tracker
cannot use them. `label_id` 11 is the static-environment class.

- **Run mode** loads a `Detection` struct with no label fields. HDF5 never
  copies those bytes into the process, which makes the separation structural
  rather than a filter that could be bypassed.
- **Eval mode** loads a separate struct that includes them, used only by
  `eval/metrics.cpp` to score tracks against ground truth.

## Open decisions

1. **Output velocity.** Report either the object's over-ground velocity
   rotated into car axes ("that car is doing 20 m/s") or its velocity
   relative to the ego ("that car is closing at 2 m/s"). Both are legitimate;
   choose deliberately before writing the output format.
2. **Clustering frame.** A rigid transform preserves distances, so clustering
   gives identical groupings in either frame. The choice affects only
   convenience, such as computing range-dependent thresholds from distance to
   the ego.
3. **Radial skew correction.** Adopt the Doppler-based correction under
   "Target-motion skew" only if clustering results show the spread matters.
