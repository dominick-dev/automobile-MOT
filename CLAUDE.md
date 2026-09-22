# Project Guidelines

See `REVIEW.md` for what automated code review should focus on and skip.

## Coordinate, unit, and timing conventions

This is a radar multi-object tracker with two distinct coordinate frames
(car frame vs. sequence frame), strict unit conventions (radians,
CCW-positive; meters; sequence-relative seconds as `double`), and
non-obvious timing/windowing rules for frame assembly.

`docs/conventions.md` is the authoritative contract for all of this,
consult it when reviewing or writing any code that touches position,
velocity, angle, or timestamp handling. Code that doesn't match it is
very likely a sign-flip, wrong-frame, or off-by-one-frame bug.
