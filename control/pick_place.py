"""
pick_place.py - vision-guided pick & place using a teach-grid, no analytic IK.

Detects a green object by HSV segmentation, interpolates servo counts for
joints ID1/ID2/ID4 from a checkerboard calibration recorded by checker_log.py
(inverse-distance weighting over the k nearest taught points), then picks the
object and places it aside.

Requires arm_serial.py (Feetech STS3215 serial-bus driver) and a
checker_grid.csv produced by checker_log.py.

RUN:  python pick_place.py
      (close any serial monitor holding the port first)

KEYS (in the camera window):
  space = pick the currently detected object, then place it aside & home
  h     = go home
  q     = quit

The object should sit on the calibrated plane for best accuracy. For a pick
off a different surface, set TABLE_OFFSET_* below and tune.
"""
import csv
import time

import cv2
import numpy as np

from arm_serial import Arm

# ---- config ----------------------------------------------------------
PORT = "/dev/ttyUSB0"
CAM_INDEX = 2
CSV = "checker_grid.csv"

HSV_LOW = np.array([40, 80, 60])
HSV_HIGH = np.array([85, 255, 255])
MIN_AREA = 300

GRIP_OPEN = 3950
GRIP_CLOSE = 3415
WRIST_ID5 = 1386                      # wrist held at its calibration value

HOVER_ID2 = -150                      # lift amount on ID2 (flip sign if it digs down)
TABLE_OFFSET_ID2 = 0                  # extra push-down for a table pick (0 = on board)
TABLE_OFFSET_ID4 = 0
DROP_BASE_SHIFT = 450                 # swing base this many counts to place aside

LIM = {1: (226, 2785), 2: (1623, 3844), 4: (403, 2641),
       5: (536, 1325), 6: (3415, 3950)}
HOME = {6: 3421, 5: 1360, 4: 370, 2: 2368, 1: 1500}
# ----------------------------------------------------------------------


def clamp(idn, v):
    lo, hi = LIM[idn]
    return max(lo, min(hi, int(round(v))))


def load_points(path):
    """Read the teach grid, drop rows with any -1, dedupe by pixel (keep last)."""
    seen = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            j = [int(r["ID1"]), int(r["ID2"]), int(r["ID4"]),
                 int(r["ID5"]), int(r["ID6"])]
            if any(v < 0 for v in j):
                continue
            key = (round(float(r["u"]), 1), round(float(r["v"]), 1))
            seen[key] = (float(r["u"]), float(r["v"]), j[0], j[1], j[2])
    pts = list(seen.values())
    print(f"loaded {len(pts)} clean calibration points")
    return pts


def interp(pts, u, v, k=4, power=2):
    """Inverse-distance weighting over the k nearest taught points."""
    arr = np.array([[p[0], p[1]] for p in pts])
    d = np.linalg.norm(arr - np.array([u, v]), axis=1)
    idx = np.argsort(d)[:k]
    w = 1.0 / (d[idx] ** power + 1e-6)
    w /= w.sum()
    id1 = sum(w[i] * pts[idx[i]][2] for i in range(len(idx)))
    id2 = sum(w[i] * pts[idx[i]][3] for i in range(len(idx)))
    id4 = sum(w[i] * pts[idx[i]][4] for i in range(len(idx)))
    return clamp(1, id1), clamp(2, id2), clamp(4, id4), float(d[idx[0]])


def detect_green(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    return (M["m10"] / M["m00"], M["m01"] / M["m00"])


def go_home(arm):
    for idn in [6, 5, 4, 2, 1]:
        arm.move(idn, HOME[idn])


def run_pick(arm, id1, id2, id4):
    print(f"  picking at ID1={id1} ID2={id2} ID4={id4}")
    arm.move(5, WRIST_ID5)
    arm.move(6, GRIP_OPEN)
    arm.move(1, id1)                                   # aim base
    arm.move(2, clamp(2, id2 + HOVER_ID2))             # hover above
    arm.move(4, id4)
    arm.move(2, clamp(2, id2 + TABLE_OFFSET_ID2))      # descend
    arm.move(4, clamp(4, id4 + TABLE_OFFSET_ID4))
    arm.move(6, GRIP_CLOSE); time.sleep(0.6)           # grasp
    arm.move(2, clamp(2, id2 + HOVER_ID2))             # lift
    arm.move(1, clamp(1, id1 + DROP_BASE_SHIFT))       # swing aside
    arm.move(6, GRIP_OPEN); time.sleep(0.4)            # release
    go_home(arm)
    print("  done")


def main():
    pts = load_points(CSV)
    u_lo, u_hi = min(p[0] for p in pts), max(p[0] for p in pts)
    v_lo, v_hi = min(p[1] for p in pts), max(p[1] for p in pts)

    arm = Arm(PORT)
    print("ping:", arm.ping(), " pose:", arm.read_all())

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(15):
        cap.read()

    print("space = pick | h = home | q = quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        uv = detect_green(frame)
        info = "no green"
        if uv:
            u, v = uv
            inside = (u_lo - 30 <= u <= u_hi + 30) and (v_lo - 30 <= v <= v_hi + 30)
            cv2.circle(frame, (int(u), int(v)), 8, (0, 0, 255), 2)
            id1, id2, id4, nd = interp(pts, u, v)
            info = (f"px=({u:.0f},{v:.0f}) -> ID1={id1} ID2={id2} ID4={id4}"
                    f"  {'OK' if inside else 'OUTSIDE taught area!'}")
        cv2.rectangle(frame, (int(u_lo), int(v_lo)), (int(u_hi), int(v_hi)),
                      (255, 0, 255), 1)
        cv2.putText(frame, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 2)
        cv2.putText(frame, "space=pick  h=home  q=quit", (10, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imshow("pick", frame)

        k = cv2.waitKey(1) & 0xFF
        if k == ord(' ') and uv:
            id1, id2, id4, nd = interp(pts, *uv)
            run_pick(arm, id1, id2, id4)
        elif k == ord('h'):
            go_home(arm)
        elif k == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    arm.close()


if __name__ == "__main__":
    main()
