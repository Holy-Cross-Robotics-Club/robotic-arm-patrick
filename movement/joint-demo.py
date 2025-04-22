#!/usr/bin/env python3

# This program is designed to ensure that the user's arm is calibrated in the same manner as our code envisions. 
# The code will have each joint move 30 degrees before returning to neutral (0 degrees) 

import hid
from controller import Controller
import numpy as np
import time
import sys

def demo_joint_movement(arm, angle_deg=25, delay=0.5):
    print("\nStarting fixed joint demo...")

    # Build unique list of joints (avoid duplicates)
    all_joints = []
    seen = set()
    for joint in arm.joints + [arm.hand, arm.gripper]:
        if joint.name not in seen:
            all_joints.append(joint)
            seen.add(joint.name)

    for joint in all_joints:
        # Invert direction for shoulder and wrist
        direction = -1 if joint.name in ["shoulder", "wrist"] else 1
        angle_rad = np.radians(direction * angle_deg)

        print(f"\nMoving '{joint.name}' to {direction * angle_deg:+}°")
        joint.set_position_radians(angle_rad)
        wait_until_all_stopped([joint])
        input("Press Enter to return to 0...")

        print(f"Returning '{joint.name}' to 0°")
        joint.set_position_radians(0)
        wait_until_all_stopped([joint])
        time.sleep(delay)

    print("\nDemo complete.")

if __name__ == "__main__":
    use_sim = "--sim" in sys.argv or "--both" in sys.argv
    use_arm = "--arm" in sys.argv or "--both" in sys.argv

    if not use_sim and not use_arm:
        print("Usage: ./joint_demo.py [--sim|--arm|--both]")
        sys.exit(1)

    arm = Controller(use_arm, use_sim)
    arm.connect()

    demo_joint_movement(arm)

    arm.disconnect()
