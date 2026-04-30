#!/usr/bin/env python3
"""
Description: Move the UFACTORY xArm 6 in a figure-8 pattern.

The figure-8 is traced in the XY plane (base rotation + shoulder) using a
Lissajous curve:
    J1 (base yaw)  = A * sin(t)
    J2 (shoulder)  = B * sin(2t)

Run from the ufactory/ directory (next to robot.conf):
    python3 figure8.py

Or pass the IP directly:
    python3 figure8.py 192.168.7.2
"""

import os
import sys
import time
import math

from xarm.wrapper import XArmAPI


# ── Configuration ─────────────────────────────────────────────────────────────

# Amplitude of the figure-8 in degrees for each joint
J1_AMPLITUDE = 45.0   # base yaw   (left/right sweep)
J2_AMPLITUDE = 20.0   # shoulder   (up/down sweep, half frequency → figure-8)

# How many full figure-8 cycles to complete
NUM_CYCLES = 3

# Steps per cycle (higher = smoother motion, slower overall)
STEPS_PER_CYCLE = 240

# Speed to move between waypoints (deg/s)
SPEED = 60

# Home/neutral position for all joints during the figure-8
# Adjust J3 (elbow) and others to a comfortable resting pose if needed
HOME_ANGLES = [0, 0, -30, 0, 30, 0]   # degrees


# ── IP Resolution ─────────────────────────────────────────────────────────────

if len(sys.argv) >= 2:
    ip = sys.argv[1]
    print(f"From command line, xArm ip = {ip}")
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        conf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.conf')
        parser.read(conf_path)
        ip = parser.get('xArm', 'ip')
        print(f"From robot.conf, xArm ip = {ip}")
    except Exception:
        ip = input('Please input the xArm ip address: ')
        if not ip:
            print('input error, exit')
            sys.exit(1)


# ── Build Waypoints ───────────────────────────────────────────────────────────

def build_figure8_waypoints(num_cycles, steps_per_cycle, j1_amp, j2_amp, base_angles):
    """
    Generate joint-angle waypoints tracing a Lissajous figure-8.

    Lissajous with frequency ratio 1:2 produces a figure-8:
        J1 = j1_amp * sin(t)
        J2 = j2_amp * sin(2t)

    All other joints are held at their base_angles values.
    """
    waypoints = []
    total_steps = num_cycles * steps_per_cycle

    for step in range(total_steps + 1):
        t = (2 * math.pi * step) / steps_per_cycle   # 0 → 2π per cycle
        j1 = base_angles[0] + j1_amp * math.sin(t)
        j2 = base_angles[1] + j2_amp * math.sin(2 * t)

        angles = [
            j1,
            j2,
            base_angles[2],
            base_angles[3],
            base_angles[4],
            base_angles[5],
        ]
        waypoints.append(angles)

    return waypoints


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    arm = XArmAPI(ip)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    print("Going to home position...")
    arm.move_gohome(wait=True)

    print(f"Moving to figure-8 start pose...")
    arm.set_servo_angle(angle=HOME_ANGLES, speed=SPEED, wait=True)
    time.sleep(0.5)

    waypoints = build_figure8_waypoints(
        num_cycles=NUM_CYCLES,
        steps_per_cycle=STEPS_PER_CYCLE,
        j1_amp=J1_AMPLITUDE,
        j2_amp=J2_AMPLITUDE,
        base_angles=HOME_ANGLES,
    )

    print(f"Starting figure-8: {NUM_CYCLES} cycle(s), {len(waypoints)} waypoints...")

    for i, angles in enumerate(waypoints):
        ret = arm.set_servo_angle(angle=angles, speed=SPEED, wait=True)
        if ret != 0:
            print(f"[WARN] set_servo_angle returned code {ret} at step {i}")

        # Pace the waypoint stream so the arm isn't flooded with commands
        time.sleep((2 * math.pi) / (STEPS_PER_CYCLE * SPEED / (J1_AMPLITUDE + 1e-6)) * 0.5)

    # Wait for the last move to finish
    time.sleep(1.0)

    print("Figure-8 complete. Returning home...")
    arm.move_gohome(wait=True)
    arm.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()