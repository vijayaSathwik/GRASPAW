"""
arm_serial.py - host-side serial link to the arm controller.

Wraps a line-based ASCII protocol spoken by the microcontroller firmware,
which does the actual Feetech STS3215 half-duplex bus transactions. The host
never touches the servo bus directly.

PROTOCOL (host -> firmware, one command per line, one line back)
  P            ping                     -> firmware banner / ack
  R <id>       read one joint           -> count, or negative on failure
  R 0          read all joints          -> five comma-separated counts
  M <id> <c>   move joint to count c    -> ack
  T <id> <on>  torque on(1) / off(0)    -> ack

The firmware prints READY once after reset; the constructor waits for it.
Reads are retried because the bus occasionally returns a short frame under
load.

USAGE
    from arm_serial import Arm
    arm = Arm("/dev/ttyUSB0")
    arm.ping()
    arm.read_all()           # [ID1, ID2, ID4, ID5, ID6]
    arm.move(1, 1500)
    arm.close()
"""
import time

import serial


class Arm:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.3)
        time.sleep(2.5)                      # let the board finish resetting
        self.ser.reset_input_buffer()
        for _ in range(20):
            if self.ser.readline().decode(errors="ignore").strip() == "READY":
                break

    def _cmd(self, line, budget=20.0):
        """Send one line, return the first non-empty reply within budget seconds."""
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < budget:
            r = self.ser.readline().decode(errors="ignore").strip()
            if r:
                return r
        return ""

    def ping(self):
        return self._cmd("P", 2)

    def read(self, i):
        """Read one joint count. Returns -1 if the bus never gave a valid frame."""
        for _ in range(5):
            r = self._cmd(f"R {i}", 1.0)
            try:
                v = int(r)
                if v >= 0:
                    return v
            except ValueError:
                pass
            time.sleep(0.05)
        return -1

    def read_all(self):
        """Read all five joint counts. Raises if the bus won't give a clean frame."""
        for _ in range(10):
            r = self._cmd("R 0", 1.0)
            try:
                vals = [int(x) for x in r.split(",")]
                if len(vals) == 5 and all(v >= 0 for v in vals):
                    return vals
            except ValueError:
                pass
            time.sleep(0.1)
        raise RuntimeError("read_all failed")

    def move(self, i, c):
        return self._cmd(f"M {i} {int(c)}")

    def torque(self, i, on):
        return self._cmd(f"T {i} {1 if on else 0}", 3)

    def close(self):
        self.ser.close()
