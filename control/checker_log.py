"""
checker_log.py - build the teach grid using a 6x9 checkerboard (freeze + click).

PROBLEM this solves: when the arm reaches in to touch the board, it covers
corners and detection fails. So we detect once with the arm out of the way,
freeze the corner positions, then touch each one.

WORKFLOW
  1) park the arm off the board so all 54 corners are visible
  2) press  f   to freeze the corner grid (stays on screen after this)
  3) click the corner you will touch (it turns red)
  4) jog the tip onto that corner
  5) press  space  to record (frozen pixel + current joint counts)
  6) click next corner, touch, space ... ~12 spread-out corners is enough
  7) press  q  to save checker_grid.csv

JOG KEYS
  1 2 4 5 6   select joint
  . / ,       coarse + / -
  ] / [       fine + / -
  t           torque off
  s           save
  q           save and quit

Requires arm_serial.py (Feetech STS3215 serial-bus driver).
RUN:  python checker_log.py
"""
import csv

import cv2
import numpy as np

from arm_serial import Arm

PORT = "/dev/ttyUSB0"
CAM_INDEX = 2
OUT_CSV = "checker_grid.csv"

COLS, ROWS = 6, 9
SQUARE_MM = 25.0
JOG_IDS = [1, 2, 4, 5, 6]
COARSE = 15
FINE = 5
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

frozen = None        # (54,2) frozen corner pixels
chosen = -1          # index of clicked corner


def find_corners(gray):
    ok, c = cv2.findChessboardCorners(
        gray, (COLS, ROWS),
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), CRIT)
    return c.reshape(-1, 2)


def on_mouse(event, x, y, flags, param):
    global chosen
    if event == cv2.EVENT_LBUTTONDOWN and frozen is not None:
        d = np.linalg.norm(frozen - np.array([x, y]), axis=1)
        chosen = int(np.argmin(d))
        print(f"  picked corner {chosen}")


def main():
    global frozen, chosen
    arm = Arm(PORT)
    print("ping:", arm.ping())
    pose = arm.read_all()
    print("start pose:", pose)

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(15):
        cap.read()

    cv2.namedWindow("checker")
    cv2.setMouseCallback("checker", on_mouse)
    rows = []

    def save():
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["corner", "u", "v", "X_mm", "Y_mm",
                        "ID1", "ID2", "ID4", "ID5", "ID6"])
            w.writerows(rows)
        print(f"  saved {len(rows)} rows -> {OUT_CSV}")

    sel = 0
    print(__doc__)
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if frozen is None:
            live = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            disp = live
            status = "LIVE - park arm off board, press f to freeze"
        else:
            disp = frozen
            status = "FROZEN - click a corner, touch it, press space"

        if disp is not None:
            for i, (u, v) in enumerate(disp):
                col = (0, 0, 255) if i == chosen else (255, 0, 255)
                cv2.circle(frame, (int(u), int(v)), 4, col, -1)
                cv2.putText(frame, str(i), (int(u) + 4, int(v) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
        else:
            cv2.putText(frame, "no checkerboard", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        for i, line in enumerate([status,
                                  f"jog ID{JOG_IDS[sel]}  chosen={chosen}  pts={len(rows)}",
                                  f"pose {pose}"]):
            cv2.putText(frame, line, (10, 26 + 26 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.imshow("checker", frame)

        k = cv2.waitKey(1) & 0xFF
        if k == 255:
            continue
        ch = chr(k)
        if ch == 'f':
            g = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if g is not None:
                frozen = g
                print("  corners FROZEN")
            else:
                print("  can't freeze - board not fully visible")
        elif ch in "12456" and int(ch) in JOG_IDS:
            sel = JOG_IDS.index(int(ch))
        elif ch == '.':
            pose[sel] += COARSE; arm.move(JOG_IDS[sel], pose[sel])
        elif ch == ',':
            pose[sel] -= COARSE; arm.move(JOG_IDS[sel], pose[sel])
        elif ch == ']':
            pose[sel] += FINE; arm.move(JOG_IDS[sel], pose[sel])
        elif ch == '[':
            pose[sel] -= FINE; arm.move(JOG_IDS[sel], pose[sel])
        elif ch == ' ':
            if frozen is None or not (0 <= chosen < len(frozen)):
                print("  freeze (f) and click a corner first")
            else:
                u, v = frozen[chosen]
                cx, cy = chosen % COLS, chosen // COLS
                X_mm, Y_mm = cx * SQUARE_MM, cy * SQUARE_MM
                live = arm.read_all()
                pose[:] = live
                rows.append((chosen, round(float(u), 2), round(float(v), 2),
                             X_mm, Y_mm, *live))
                print(f"  REC corner {chosen}: px=({u:.1f},{v:.1f}) "
                      f"mm=({X_mm},{Y_mm}) joints={live}  total={len(rows)}")
        elif ch == 't':
            arm.torque(0, 0)
            print("  torque toggled (sent OFF) - send a jog to re-enable")
        elif ch == 's':
            save()
        elif ch == 'q':
            save()
            break

    cap.release()
    cv2.destroyAllWindows()
    arm.close()


if __name__ == "__main__":
    main()
