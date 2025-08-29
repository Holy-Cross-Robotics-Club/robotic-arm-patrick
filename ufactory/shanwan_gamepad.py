#!/usr/bin/env python3
import hid
import time
from collections import namedtuple
import os
import sys
import time
import math
from xarm.wrapper import XArmAPI

VENDOR_ID  = 0x2563
PRODUCT_ID = 0x0575

# Show diffs vs baseline? (set True while mapping your bits)
RAW_DUMP   = False

# Your idle packet:
BASELINE = bytes.fromhex(
    "00 00 0f 80 80 80 80 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 02 00 02 00 02 00 02"
)

# ---- Configure which bits in BYTE 0 correspond to your virtual axes ----
# Set these once you learn them (see console diff messages).
# Example: if bit 2 is D-Pad LEFT and bit 3 is D-Pad RIGHT, set:
#   B0_LEFT=2; B0_RIGHT=3
RY_DOWN  = 0
RX_RIGHT = 1
RY_UP    = 2
RX_LEFT  = 3
B0_RIGHT = 4
B0_DOWN  = 5
B0_LEFT  = 6
B0_UP    = 7

# How hard to push the virtual axis when a button is pressed
VAXIS_VALUE = 1.0

Axes = namedtuple("Axes", "lx ly rx ry ax5 ax6")

def list_devices():
    for d in hid.enumerate():
        if d["vendor_id"] == VENDOR_ID and d["product_id"] == PRODUCT_ID:
            return d
    return None

def to_signed_norm(v, center=128, span=127.0):
    v = max(0, min(255, v))
    return (v - center) / span

def bit(v, i):
    return (v >> i) & 1

def synth_axis_from_bits(b, neg_bit, pos_bit, value=VAXIS_VALUE):
    """Return -value, 0, +value based on two opposing bits in byte b.
       If both pressed (or both None), returns 0 to avoid ambiguity."""
    if neg_bit is None or pos_bit is None:
        return 0.0
    n = bit(b, neg_bit)
    p = bit(b, pos_bit)
    if n and not p:
        return -value
    if p and not n:
        return +value
    return 0.0

def parse_report(data):
    # We only need first few bytes for this layout
    if len(data) < 5:
        return None

    b0 = data[0]         # buttons bitfield (on your unit)
    lx = data[3]
    ly = data[4]

    # Sticks (right stick seems fixed at 0x80 on your pad; keep them centered)
    lx_n = to_signed_norm(lx)
    ly_n = -to_signed_norm(ly)
    rx_n = synth_axis_from_bits(b0, RX_LEFT, RX_RIGHT)
    ry_n = synth_axis_from_bits(b0, RY_UP,   RY_DOWN)

    # Two extra axes synthesized from button pairs in BYTE 0
    ax5 = synth_axis_from_bits(b0, B0_LEFT, B0_RIGHT)
    ax6 = synth_axis_from_bits(b0, B0_UP,   B0_DOWN)

    return Axes(lx_n, ly_n, rx_n, ry_n, ax5, ax6)

def dump_b0_diffs(prev_b0, cur_b0):
    if prev_b0 == cur_b0:
        return
    changed = prev_b0 ^ cur_b0
    flips = [i for i in range(8) if (changed >> i) & 1]
    if flips:
        states = " ".join(f"b0[{i}]={'1' if (cur_b0>>i)&1 else '0'}" for i in flips)
        print(f"B0 change: {states} (b0=0x{cur_b0:02x})")

def main():
    devinfo = list_devices()
    if not devinfo:
        print("Device 2563:0575 not found. Is the dongle plugged in?")
        return

    print(f"Opening: {devinfo.get('product_string') or 'USB WirelessGamepad'} "
          f"{devinfo['vendor_id']:04x}:{devinfo['product_id']:04x}")

    h = hid.device()
    h.open(VENDOR_ID, PRODUCT_ID)
    h.set_nonblocking(False)

    print("Reading reports... Press Ctrl+C to quit.")
    last_axes = None
    last_b0 = None
    t0 = time.time()


    # Resolve IP the same way your sample does (env/config/arg). For brevity:

    if len(sys.argv) >= 2:
        ip = sys.argv[1]
        print(f"From command line, xArm ip = {ip}")
    else:
        try:
            from configparser import ConfigParser
            parser = ConfigParser()
            parser.read('./robot.conf')
            ip = parser.get('xArm', 'ip')
            print(f"From ./robot.conf, xArm ip = {ip}")
        except:
            ip = input('Please input the xArm ip address:')
            if not ip:
                print('input error, exit')
                sys.exit(1)
    ROBOT_IP = ip
    ctrl = ArmController(ROBOT_IP, step_deg_per_sec=20.0, deadband=0.15, require_enable=False)

    try:
        while True:
            data = h.read(64, timeout_ms=250) if hasattr(h, "read") else None
            if not data:
                continue

            if RAW_DUMP:
                # Only print diffs vs baseline, and specifically show byte 0 bit flips
                diffs = [(i, b) for i, b in enumerate(data)
                         if i >= len(BASELINE) or b != BASELINE[i]]
                if diffs:
                    # Highlight byte 0 changes first
                    for i, b in diffs:
                        if i == 0:
                            dump_b0_diffs(last_b0 if last_b0 is not None else b, b)
                            last_b0 = b
                    # Then list other differing bytes (besides 0, 3, 4 which we expect)
                    other = [(i, b) for i, b in diffs if i not in (0, 3, 4)]
                    if other:
                        diff_str = " ".join(f"{i}:{b:02x}" for i, b in other)
                        print(f"DIFF (non-[0,3,4]) -> {diff_str}")

            axes = parse_report(data)
            if not axes:
                continue

            if axes != last_axes or (time.time() - t0) > 0.25:
                last_axes = axes
                t0 = time.time()
                print(
                    f"LX={axes.lx:+.2f}  LY={axes.ly:+.2f}  "
                    f"RX={axes.rx:+.2f}  RY={axes.ry:+.2f}  "
                    f"AX5={axes.ax5:+.2f} AX6={axes.ax6:+.2f}"
                )
                ctrl.update_from_axes(axes)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print("\nShutting down xArm.")
        ctrl.shutdown()
        print("\nClosing hid input.")
        h.close()
        print("\nDone.")


class ArmController:
    def __init__(self, ip, step_deg_per_sec=15.0, deadband=0.2, require_enable=False):
        """
        step_deg_per_sec: how many degrees per second to move at full input (|axis|=1)
        deadband: ignore |axis| < deadband
        require_enable: if True, motion only when self.enabled is True (you can toggle this via a button)
        """
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        # seed targets from current pose (in radians)
        ok, joints = self.arm.get_servo_angle(is_radian=True)
        if ok != 0:
            # fallback to zeros if read failed
            joints = [0.0] * 6
        self.target = list(joints[:6]) + [0.0] * max(0, 6 - len(joints))
        self.deadband = deadband
        self.rad_per_sec = math.radians(step_deg_per_sec)
        self.last_t = time.time()
        self.enabled = not require_enable

        # gentle speed limit for the arm (rad/s). This is separate from per-step increments.
        self.stream_speed = math.radians(60)  # tweak if you want faster/slower overall motion

    def _apply_deadband(self, v):
        return 0.0 if abs(v) < self.deadband else (1.0 if v > 0 else -1.0)

    def set_enabled(self, flag: bool):
        self.enabled = flag

    def shutdown(self):
        try:
            self.arm.move_gohome(wait=True)
        finally:
            self.arm.disconnect()

    def update_from_axes(self, axes):
        """
        axes: namedtuple Axes(lx, ly, rx, ry, ax5, ax6) with each in {-1,0,1} approx
        Call this every loop after you read/print the axes.
        """
        now = time.time()
        dt = max(0.0, min(0.2, now - self.last_t))  # clamp dt to avoid big jumps if paused
        self.last_t = now

        if not self.enabled:
            return

        # Map your 6 axes to the 6 joints (J1..J6). Tweak signs to taste.
        # Common choice:
        #   J1 (base yaw)     <- LX
        #   J2 (shoulder)     <- -LY   (invert so pushing stick up raises arm)
        #   J3 (elbow)        <- RX
        #   J4 (wrist pitch)  <- -RY
        #   J5 (wrist yaw)    <- AX5
        #   J6 (wrist roll)   <- AX6
        a = [
            self._apply_deadband(axes.lx),
            self._apply_deadband(-axes.ly),
            self._apply_deadband(axes.rx),
            self._apply_deadband(-axes.ry),
            self._apply_deadband(axes.ax5),
            self._apply_deadband(axes.ax6),
        ]

        # Convert to incremental radians using dt and max rate
        step = self.rad_per_sec * dt
        for j in range(6):
            self.target[j] += a[j] * step

        # (Optional) soft clamps if you want—commented out to let the controller be permissive.
        # def clamp(x, lo, hi): return min(hi, max(lo, x))
        # self.target[0] = clamp(self.target[0], math.radians(-170), math.radians(170))
        # self.target[1] = clamp(self.target[1], math.radians(-120), math.radians(120))
        # self.target[2] = clamp(self.target[2], math.radians(-225), math.radians(11))
        # self.target[3] = clamp(self.target[3], math.radians(-180), math.radians(180))
        # self.target[4] = clamp(self.target[4], math.radians(-180), math.radians(180))
        # self.target[5] = clamp(self.target[5], math.radians(-360), math.radians(360))

        # Stream the new target. wait=False so we can keep updating each loop.
        # Note: speed is interpreted in rad/s when is_radian=True.
        self.arm.set_servo_angle(
            angle=self.target,
            is_radian=True,
            speed=self.stream_speed,
            wait=False
        )


if __name__ == "__main__":
    main()

