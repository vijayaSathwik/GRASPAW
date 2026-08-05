# Control and vision

Teach-grid pick-and-place for the GRASPAW arm. No analytic inverse kinematics:
the mapping from camera pixels to servo counts is measured on the physical arm
and interpolated.

## Files

| File | What it does |
|---|---|
| `arm_serial.py` | Feetech STS3215 serial-bus driver (move, read, ping, torque) |
| `checker_log.py` | Records the teach grid against a 6x9 checkerboard -> `checker_grid.csv` |
| `pick_place.py` | Detects a green object, interpolates joint counts, picks and places |

## How the mapping works

`checker_log.py` freezes the detected checkerboard corners while the arm is out
of frame, then you jog the tip onto each clicked corner and record the pixel
position alongside the servo counts. That gives a set of (pixel -> joint counts)
pairs measured on the real hardware, so link-length error, servo backlash and
calibration offsets are baked into the data rather than modelled.

`pick_place.py` then takes a detected object centroid and blends the four
nearest taught points by inverse-distance weighting (k=4, p=2) to get the
joint counts for that pixel.

The trade: the grid is valid only for this physical arm. Replace a servo or a
link and it must be retaught.

## Running

    python checker_log.py     # build checker_grid.csv first
    python pick_place.py      # then pick

Both need the serial port free — close any serial monitor first. Port and
camera index are set at the top of each file.

## Status

Thesis code. It works on the hardware it was written for; it was not written
to be read by anyone else. Being cleaned up as the arm is rebuilt.
